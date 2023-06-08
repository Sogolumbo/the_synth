import numpy as np

def saw(i, length):
    return 127-(i%256);
def triangle(i, length):
    if(i<(length/4)):
        return 2*i
    elif(i < length*3/4):
        return 256-2*i
    else:
        return -512 +2*i

filename = 'wavetable_output.txt' 
with open(filename, 'w') as f:

    
    if(False): #Attack and Decay
        f.write('\n')
        for i in range(128):
            f.write("\t{},\t//{}\n".format(i*2+1, i))
        f.write("\n");
        for i in range(128):
            f.write("\t{},\t//{}\n".format(254-i*2, 128+i))

            
    elif(True): #Saw and Triangle waveform mix
        f.write("{\n")
        length = 256
        flavours = 16
        for j in range(flavours):
            f.write(" { ")
            for i in range(length):
                f.write("{:.0f},".format( (j)/(flavours-1)*saw(i, length) +
                                           (flavours-1-j)/31*triangle(i, length) ))
                if(i%32 == -1%32):
                    f.write("\n   ")
            f.write("},\n")
        f.write("};")
print("Saved file as: {}".format(filename))
