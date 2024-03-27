#!/usr/bin/ pyhton3.8

import subprocess as sp

if __name__ == '__main__':
    l=[]
    with open('./input.txt', 'r') as read_file:
        for line in read_file:
            if "nh.param" in line:
                string = f'/{line.split(",")[0].split("(")[1][1:-1]}\n'
                if string not in l: l.append(string)

    l.sort()

    with open('mpc_parameter_list.txt', 'w') as write_file:
        write_file.writelines(l)