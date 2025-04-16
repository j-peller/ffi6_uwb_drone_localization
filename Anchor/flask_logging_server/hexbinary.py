def print_bits_from_hex(hex_value):
    number = int(hex_value, 16)
    bit_length = max(8, ((number.bit_length() + 7) // 8) * 8)
    bit_string = format(number, f'0{bit_length}b')

    # Bit-Zeichenkette mit Abstand
    bit_line = '|'.join(f'{b:>2}' for b in bit_string)

    # Bitnummern mit gleicher Breite (rechtsbündig für 2-stellige Zahlen)
    bit_numbers = [f'{i:>2}' for i in range(bit_length - 1, -1, -1)]
    bit_number_line = '|'.join(bit_numbers)

    print("Bits:        " + bit_line)
    print("Bitnummern:  " + bit_number_line)
if __name__ == "__main__":
    hex_input = input("Hexadezimalzahl eingeben (z. B. 3F oder 0x3F): ").strip().lower()
    if hex_input.startswith("0x"):
        hex_input = hex_input[2:]
    
    print_bits_from_hex(hex_input)