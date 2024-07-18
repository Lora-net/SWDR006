import sys

try:
    fn = sys.argv[1]
except IndexError:
    print("please supply filename to search")
    sys.exit(-1)

print('search file ', fn)
# string to search in file
with open(fn, 'r') as fp:
    # read all lines using readline()
    lines = fp.readlines()
    word = 'lr11xx'
    for row in lines:
        # check if string present on a current line
        #print(row.find(word))
        # find() method returns -1 if the value is not found,
        # if found it returns index of the first occurrence of the substring
        if row.find(word) != -1:
            print('string exists in file')
            print('line Number:', lines.index(row))
            sys.exit(0)
    print(word, ' not found')
    sys.exit(1) # indicate not found

