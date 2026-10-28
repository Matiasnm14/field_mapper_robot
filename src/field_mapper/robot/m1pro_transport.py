"""ASCII TCP cuatro ejes: límites, fragmentos y respuesta ligada a su petición."""

from dataclasses import dataclass
import re
import socket
import threading
import time

from ..errors import ConnectionLost, MotionError, MotionTimeout


@dataclass(frozen=True)
class Reply:
    error_id: int
    payload: str
    command: str
    raw: str


def _command_parts(command):
    match = re.fullmatch(r"([A-Za-z][A-Za-z0-9]*)\(([^;{}()]*)\)", command.strip())
    if match is None:
        raise MotionError("Comando o eco TCP/IP mal formado.")
    body = match.group(2).strip()
    arguments = [value.strip() for value in body.split(",")] if body else []
    if any(not value for value in arguments):
        raise MotionError("El eco TCP/IP contiene un argumento vacío.")
    return match.group(1).casefold(), arguments


def parse_reply(raw: str, expected_command: str) -> Reply:
    match = re.fullmatch(r"\s*(-?\d+)\s*,\s*\{([^{}]*)\}\s*,\s*([^;]+);\s*", raw)
    if match is None:
        raise MotionError("Respuesta ASCII TCP/IP mal formada.")
    echo = match.group(3).strip()
    name, arguments = _command_parts(echo)
    expected_name, expected_arguments = _command_parts(expected_command)
    if name != expected_name:
        raise MotionError("El eco de la respuesta no corresponde a la petición actual.")
    # El manual muestra GetPose() y MovL(X,Y,Z,R) sin opciones en el eco.
    required = 4 if name == "movl" else (0 if name == "getpose" else len(expected_arguments))
    allowed_lengths = {required, len(expected_arguments)} if name in {"getpose", "movl"} else {required}
    if len(arguments) not in allowed_lengths:
        raise MotionError("El eco no contiene los argumentos necesarios.")
    for actual, expected in zip(arguments, expected_arguments):
        try:
            equal = float(actual) == float(expected)
        except ValueError:
            equal = actual.casefold().replace(" ", "") == expected.casefold().replace(" ", "")
        if not equal:
            raise MotionError("Los argumentos del eco difieren de la petición.")
    if name not in {"getpose", "movl"} and len(arguments) != len(expected_arguments):
        raise MotionError("Cantidad de argumentos de eco incompatible.")
    reply = Reply(int(match.group(1)), match.group(2).strip(), echo, raw)
    if reply.error_id:
        raise MotionError(f"El controlador rechazó {expected_name}: ErrorID={reply.error_id}.")
    return reply


class ASCIIChannel:
    """Un único intercambio por canal; tras incertidumbre no se reutiliza."""

    def __init__(self, handle, *, monotonic=time.monotonic, max_message_bytes=65536):
        self.handle = handle
        self.monotonic = monotonic
        self.max_message_bytes = max_message_bytes
        self.buffer = bytearray()
        self.closed = False
        self._lock = threading.Lock()

    def close(self):
        if not self.closed:
            self.closed = True
            self.handle.close()

    def _remaining(self, deadline):
        remaining = deadline - self.monotonic()
        if remaining <= 0:
            raise MotionTimeout("Plazo de comunicación TCP/IP agotado.")
        self.handle.settimeout(remaining)

    def read_message(self, *, deadline):
        while True:
            self._remaining(deadline)
            end = self.buffer.find(b";")
            if end >= 0:
                message = bytes(self.buffer[:end + 1])
                del self.buffer[:end + 1]
                try:
                    return message.decode("ascii")
                except UnicodeDecodeError as error:
                    raise MotionError("Respuesta TCP/IP fuera del esquema ASCII.") from error
            data = self.handle.recv(4096)
            if not data:
                raise ConnectionLost("recv vacío: conexión TCP/IP cerrada por el controlador.")
            self.buffer.extend(data)
            if len(self.buffer) > self.max_message_bytes:
                raise MotionError("Respuesta TCP/IP supera el tamaño permitido.")

    def request(self, command: str, *, deadline: float) -> Reply:
        remaining = deadline - self.monotonic()
        if remaining <= 0 or not self._lock.acquire(timeout=remaining):
            raise MotionTimeout("Plazo agotado esperando exclusión del canal TCP/IP.")
        try:
            if self.closed:
                raise ConnectionLost("Canal TCP/IP cerrado.")
            try:
                if self.buffer.strip():
                    raise MotionError("Respuesta no solicitada pendiente; no acredita frescura.")
                self.buffer.clear()
                self._remaining(deadline)
                self.handle.sendall(command.encode("ascii"))
                raw = self.read_message(deadline=deadline)
                self._remaining(deadline)
                if self.buffer.strip():
                    raise MotionError("Respuestas concatenadas no solicitadas; no acreditan frescura.")
                return parse_reply(raw, command)
            except (socket.timeout, TimeoutError) as error:
                self.close()
                raise MotionTimeout("Timeout TCP/IP; resultado del comando incierto, no reenviar.") from error
            except OSError as error:
                self.close()
                raise ConnectionLost(f"Conexión TCP/IP perdida: {error}.") from error
            except BaseException:
                self.close()
                raise
        finally:
            self._lock.release()
