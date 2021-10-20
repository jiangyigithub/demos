def save_params(path, args):
    with open(path + "/params.log", 'w') as paramlog:
        for key in vars(args):
            info = key + ": " + str(getattr(args,key))
            print('\t' + info)
            paramlog.write(info + '\n')



def get_cloud_size(pcd):
    # open file and check the number of points
    with open(pcd, 'rb') as f:
        dat = f.readlines()
        dat = dat[6].decode('utf-8')

        return int(dat.strip().split(' ')[1])

