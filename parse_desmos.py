
keep = list('1234567890[](),')

s = input('What you got from desmos:\n')
s = ''.join([c for c in list(s) if c in keep])
s = s.replace('(', '[')
s = s.replace(')', ']')
obj = eval(s)
obj = [[x[0], x[2], x[1]] for x in obj]
print()
print(obj)
