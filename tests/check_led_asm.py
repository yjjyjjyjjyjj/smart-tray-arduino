"""Check the actual optimized UNO ELF, not just the C++ constraints.
Usage: python tests/check_led_asm.py <avr-objdump.exe> <firmware.elf>
"""
import re
import subprocess
import sys

dump = subprocess.check_output([sys.argv[1], '-d', '-C', sys.argv[2]], text=True)
loops = re.findall(r'^\w+ <head20_\d+>:\n(.*?)^\w+ <nextbyte20_\d+>:', dump, re.M | re.S)
assert loops, 'No WS2812 loop found'
for loop in loops:
    instructions = []
    for line in loop.splitlines():
        m = re.match(r'\s*[0-9a-f]+:\s+(?:[0-9a-f]{2}\s+)+\s*(\w+)\s*(.*?)\s*(?:;.*)?$', line)
        if m:
            instructions.append((m[1], m[2].split(';')[0].strip()))
    # st port, hi; sbrc byte,7; mov next,hi; dec bit; st port,next; mov next,lo
    assert [x[0] for x in instructions[:6]] == ['st','sbrc','mov','dec','st','mov'], instructions
    high = instructions[0][1].split(',')[1].strip()
    scratch, low = [r.strip() for r in instructions[5][1].split(',')]
    assert scratch != low, 'BROKEN: next aliases lo; reset becomes mov rN,rN'
    assert low != high, 'BROKEN: high and low share a register'
    assert instructions[4][1].split(',')[1].strip() == scratch
    assert instructions[9][0] == 'st' and instructions[9][1].split(',')[1].strip() == low
    for op, args in instructions:
        if op in ('mov','dec','adc','rol','ldi','ld','sbiw'):
            assert args.split(',')[0].strip() != low, 'LOW register overwritten in bit loop'
    print(f'PASS: compiled WS2812 loop keeps next={scratch}, low={low}, high={high} distinct')
