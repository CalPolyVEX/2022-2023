# python c++_style_check.py <fileDirectory>

#
# A simple-minded style checker for C++ code.
# This only catches the most obvious style mistakes, and occasionally
# flags stuff that isn't wrong.
#

from colorama import init, Fore
import sys, os, re, textwrap


init(autoreset=True)

#
# Constants.
#

MAX_LINE_LENGTH = 200

#
# Regular expressions corresponding to style violations.
#

tabs = re.compile(r'\t+')
comma_space = re.compile(',[^ ]\n')

# This one is really tough to get right, so we settle for catching the
# most common mistakes.  Add other operators as necessary and/or feasible.
operator_space = re.compile('(\w(\+|\-|\*|\<|\>|\=)\w)' + '|(\w(\=\=|\<\=|\>\=)\w)')
comment_line = re.compile('^\s*\/\*.*\*\/\s*$')
cpp_comment_line = re.compile('^\s*\/\/.*$')
open_comment_space = re.compile('\/\*[^ *\n]\n')
close_comment_space = re.compile('[^ *]\*\/')
cpp_comment_space = re.compile('\/\/[^ *\n]\n')
paren_curly_space = re.compile('\)\{')
trailing_curly_space = re.compile('\)\n\{')
semi_space = re.compile(';[^ \s]\n')

errorcount = [0, 0]

def check_line(filename, line, n, errorcount, nextline=None):
  '''
  Check a line of C++ code for style mistakes. Comments are ignored, and are assumed to be properly formatted. 
  '''
  # Strip the terminal newline.
  line = line[:-1]

  basename = os.path.basename(filename)
  if (line.startswith('/**') or line.startswith(' *') or line.startswith(' */')):
    pass
  else:
    if tabs.search(line):
      print(Fore.YELLOW + f'File "{filename}", line {n}, in {basename}\n{line}\nWarning: Line contains tab characters')
      errorcount[1] += 1
    if len(line) > MAX_LINE_LENGTH:
      print(Fore.YELLOW + f'File "{filename}", line {n}, in {basename}\n{line}\nWarning: Line contains too many character ({len(line)} characters found, maximum should be {MAX_LINE_LENGTH})\n')
      errorcount[1] += 1
    if comma_space.search(line):
      print(Fore.LIGHTRED_EX + f'File "{filename}", line {n}, in {basename}\n{line}\nError: Place a space after ","\n')
      errorcount[0] += 1
    if operator_space.search(line):
      if not(comment_line.search(line)) and not(cpp_comment_line.search(line)):
        print(Fore.LIGHTRED_EX + f'File "{filename}", line {n}, in {basename}\n{line}\nError: Place a space around operators\n')
        errorcount[0] += 1
    if open_comment_space.search(line):
      print(Fore.LIGHTRED_EX + f'File "{filename}", line {n}, in {basename}\n{line}\n[PUT SPACE AFTER OPEN COMMENT]\n')
      errorcount[0] += 1
    if close_comment_space.search(line):
      print(Fore.LIGHTRED_EX + f'File "{filename}", line {n}, in {basename}\n{line}\n[PUT SPACE BEFORE CLOSE COMMENT]\n')
      errorcount[0] += 1
    if cpp_comment_space.search(line):
      print(Fore.LIGHTRED_EX + f'File "{filename}", line {n}, in {basename}\n{line}\n[PUT SPACE AFTER OPEN COMMENT]\n')
    if paren_curly_space.search(line):
      print(Fore.LIGHTRED_EX + f'File "{filename}", line {n}, in {basename}\n{line}\nError: Place a space between ")" and "\u007b"\n')
      errorcount[0] += 1
    if semi_space.search(line):
      print(Fore.LIGHTRED_EX + f'File "{filename}", line {n}, in {basename}\n{line}\n[PUT SPACE/NEWLINE AFTER SEMICOLON]\n')
      errorcount[0] += 1
    if nextline is None:
      pass
    else:
      if trailing_curly_space.search(f"{line}\n{textwrap.dedent(nextline)}"):
        print(Fore.LIGHTRED_EX + f'File "{filename}", lines {n}-{n + 1}, in {basename}\n{line}\n{nextline}\nError: K&R style is used in this project. Place "\u007b" a space after ")"\n')
        errorcount[0] += 1
  return errorcount

def check_file(filename, errorcount):
  file = open(filename, 'r')
  lines = file.readlines()
  file.close()

  for i in range(len(lines)):
    if i == len(lines) - 1:
      check_line(filename, lines[i], i + 1, errorcount)  # Start on line 1.
    else:
      check_line(filename, lines[i], i + 1, errorcount, nextline=(lines[i + 1]))
  return errorcount

usage = 'usage: c++_style_check filename1 [filename2 ...]'

if len(sys.argv) < 2:
  print(Fore.LIGHTRED_EX + usage)
  raise SystemExit

for filename in sys.argv[1:]:
  check_file(filename, errorcount)

if errorcount[0] + errorcount[1] == 0:
  print(Fore.GREEN + 'Looks good!')
else:
  print(Fore.YELLOW + f'{errorcount[0]} error(s) found. {errorcount[1]} warning(s) found.')
