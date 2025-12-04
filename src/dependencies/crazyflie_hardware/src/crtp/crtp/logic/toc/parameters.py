import struct 
import logging
logger = logging.getLogger(__name__)

class ParamTocElement:
    """An element in the Log TOC."""

    RW_ACCESS = 0
    RO_ACCESS = 1

    EXTENDED_PERSISTENT = 1

    types = {0x08: ('uint8_t', '<B'),
             0x09: ('uint16_t', '<H'),
             0x0A: ('uint32_t', '<L'),
             0x0B: ('uint64_t', '<Q'),
             0x00: ('int8_t', '<b'),
             0x01: ('int16_t', '<h'),
             0x02: ('int32_t', '<i'),
             0x03: ('int64_t', '<q'),
             0x05: ('FP16', ''),
             0x06: ('float', '<f'),
             0x07: ('double', '<d')}

    def __init__(self, ident=0, data=None):
        """TocElement creator. Data is the binary payload of the element."""
        self.ident = ident
        self.persistent = False
        self.extended = False
        if (data):
            strs = struct.unpack('s' * len(data[1:]), data[1:])
            s = ''
            for ch in strs:
                s += ch.decode('ISO-8859-1')
            strs = s.split('\x00')
            self.group = strs[0]
            self.name = strs[1]

            metadata = data[0]
            if isinstance(metadata, str):
                metadata = ord(metadata)

            # If the fouth byte (1 << 4) (0x10) is set we have extended
            # type information for this element.
            self.extended = ((metadata & 0x10) != 0)

            self.ctype = self.types[metadata & 0x0F][0]
            self.pytype = self.types[metadata & 0x0F][1]
            if ((metadata & 0x40) != 0):
                self.access = ParamTocElement.RO_ACCESS
            else:
                self.access = ParamTocElement.RW_ACCESS

    def get_readable_access(self):
        if (self.access == ParamTocElement.RO_ACCESS):
            return 'RO'
        return 'RW'

    def is_extended(self):
        return self.extended

    def mark_persistent(self):
        self.persistent = True

    def is_persistent(self):
        return self.persistent
