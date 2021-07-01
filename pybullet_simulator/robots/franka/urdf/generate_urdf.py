

def list_files(dir):
    r = []
    subdirs = [x[0] for x in os.walk(dir)]
    for subdir in subdirs:
        files = os.walk(subdir).next()[2]
        if (len(files) > 0):
            for file in files:
                r.append(os.path.join(subdir, file))
    return r

if __name__ == "__main__":
    print('start processing...')

    import os
    cwd = os.getcwd()
    print('cwd: ', cwd)

    files = list_files(cwd)
    print('num of files: ', len(files))
    # for file in files:
    #     print(file)

    for f in files:
        if f.endswith('.xacro'):
            print(f)


            # convert dae to obj
            bashCommand = 'rosrun xacro xacro ' + f + ' > ' + f.replace('.xacro','.urdf')
            print('bashCommand: ', bashCommand)
            import subprocess
            process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
            output, error = process.communicate()

            # # Read in the file
            # with open(f, 'r') as file:
            #     filedata = file.read()
            #
            # # Replace the target string
            # filedata = filedata.replace('model://', 'package://models/')
            # filedata = filedata.replace('visual.dae', 'visual.obj')
            #
            # # Write the file out again
            # with open(f, 'w') as file:
            #   file.write(filedata)

