
# Attack & Decay

filename = 'wavetable_output.txt' 
with open(filename, 'w') as f:
    f.write('\n')
    for i in range(128):
        f.write("\t{},\t//{}\n".format(i*2+1, i));
    f.write("\n");
    for i in range(128):
        f.write("\t{},\t//{}\n".format(255-i*2, 128+i));
print("Saved file as: {}".format(filename))
