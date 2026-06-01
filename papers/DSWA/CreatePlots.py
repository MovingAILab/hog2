import numpy as np 
import sys
import matplotlib.pyplot as plt
import scipy.stats as st 
from collections import OrderedDict


plt.rcParams["figure.figsize"] = [7.00, 7.00]
plt.rcParams["legend.framealpha"] = 1.00
## Args: PythonAdrress Domain #Experiment #Policies #Weights DataAdrress

weight_to_int = {"1.50":0, "2.00":1, '3.00':2, '4.00':3, '5.00':4, '6.00':5, '7.00':6, '8.00':7, '9.00':8, '10.00':9}


if int(sys.argv[3])==8:
    markers = [',', '^', 'X', '8', 's', '*', 'o', 'd'] ##DWP=5, MAP=6, DPS=7
    int_to_alg = {0:'WA*', 1:'PWXD', 2:'PWXU', 3:'XDP', 4:'XUP', 5:'DWP', 6:'MAP', 7:'DPS'}
    colours = ['tab:gray', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:blue', 'tab:cyan', 'tab:olive']
if int(sys.argv[3])==7:
    markers = [',', '^', 'X', '8', 's', '*', 'o'] ##DWP=5, MAP=6
    int_to_alg = {0:'WA*', 1:'PWXD', 2:'PWXU', 3:'XDP', 4:'XUP', 5:'DWP', 6:'MAP'}
    colours = ['tab:gray', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:blue', 'tab:cyan']


linestyles = OrderedDict(
    [('dashed',              (0, (5, 5))),
     ('dashdotted',          (0, (3, 5, 1, 5))),
     ('dotted',              (0, (1, 5))),
     ('long dash with offset', (5, (10, 3))),
     ('dashdotdotted',         (0, (3, 5, 1, 5, 1, 5))),
     ('solid',               (0, ())),
     ('loosely dotted',      (0, (1, 10))),
     ('densely dashed',      (0, (5, 1))),
     ('densely dotted',      (0, (1, 1))),
     ('loosely dashed',      (0, (5, 10))),
     ('loosely dashdotted',  (0, (3, 10, 1, 10))),
     ('densely dashdotted',  (0, (3, 1, 1, 1))),
     ('loosely dashdotdotted', (0, (3, 10, 1, 10, 1, 10))),
     ('densely dashdotdotted', (0, (3, 1, 1, 1, 1, 1)))])

showErrorBar = False

if sys.argv[1] == '-stp':
    if sys.argv[2] == '1':

        ##Experiment 1: Creates a table from average of runs of different problems over different weights
        table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        count_table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        TheDataSet = [[[] for _ in range(int(sys.argv[4]))] for _ in range(int(sys.argv[3]))]

        with open("./localJobs/OutputData/"+sys.argv[5]+".txt", "r") as f:
            for line in f:
                data = line.split()
                if(len(data)): ## To check for empy lines
                    if data[0] == "STP" and (data[5] in list(weight_to_int.keys())) and (int(data[3]) in list(int_to_alg.keys())) : #and data[3]!='0' and data[3]!='1' and data[3]!='6' and data[3]!='8': # and data[9]!='0':
                        table[int(data[3])][weight_to_int[data[5]]] += int(data[7])
                        count_table[int(data[3])][weight_to_int[data[5]]] += 1
                        TheDataSet[int(data[3])][weight_to_int[data[5]]].append(int(data[7]))
                        # print(float(data[7]), count_table[int(data[3])][weight_to_int[data[5]]], sep=" ")

        result = np.divide(table, count_table)

        print()
        print('====================================================  Average Expansions Table  =====================================================')
        print('Alg / Weight|   1.50    |   2.00    |   3.00    |   4.00    |   5.00    |   6.00    |   7.00    |   8.00    |   9.00    |   10.0    |')
        print('_________________' * 7)
        for i in range(len(table)):
            if i!=-1:
                print(int_to_alg[i],end="")
                for k in range(12-len(int_to_alg[i])):
                    print(' ',end="")
                print('|', end="")
                for j in range(len(table[i])):
                    print(round(result[i][j], 2), end="")
                    for k in range(11-len(str(round(result[i][j], 2)))):
                        print(' ',end="")
                    print('|', end="")
                print()
        print('__________________' * 7)
        for cnt in range(len(table)):
            print(str(cnt+1)+' place Alg |', end="")
            for i in range(len(table[0])):
                col = table[:,i]
                print(int_to_alg[np.argsort(col)[cnt]], end="")
                # print(int_to_alg[np.argmin(col)], end="")
                for k in range(11-len(int_to_alg[np.argsort(col)[cnt]])):
                    print(' ',end="")
                print('|', end="")
            print()
        
        print('=====================================================',end='')
        for _ in range((28 - len(sys.argv[5]))//2):
            print(' ', end='')
        print(sys.argv[5], end='')
        for _ in range((28 - len(sys.argv[5]))//2):
            print(' ', end='')
        print('====================================================')
        print()

    ##############################################
    elif sys.argv[2] == '2':

        ##Experiment 2: Creates the work/weight plot
        ##arg[3] is the number of algs and arg[4] is the number of weights
        table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        count_table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        TheDataSet = [[[] for _ in range(int(sys.argv[4]))] for _ in range(int(sys.argv[3]))]

        with open("./localJobs/OutputData/"+sys.argv[5]+".txt", "r") as f:
            for line in f:
                data = line.split()
                if(len(data)): ## To check for empy lines
                    if data[0] == "STP" and (data[5] in list(weight_to_int.keys())) and (int(data[3]) in list(int_to_alg.keys())) : #and data[3]!='0' and data[3]!='1' and data[3]!='6' and data[3]!='8': # and data[9]!='0':
                        table[int(data[3])][weight_to_int[data[5]]] += float(data[7])
                        count_table[int(data[3])][weight_to_int[data[5]]] += 1
                        TheDataSet[int(data[3])][weight_to_int[data[5]]].append(int(data[7]))

        result = np.divide(table, count_table)

        Weights = [1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
        
        xpoints = np.array(Weights)
        works = []
        for policy in int_to_alg:
            work_i = []
            for w in list(weight_to_int.keys()):
                work_i.append(result[policy][weight_to_int[w]])
            works.append(work_i)
        for policy in int_to_alg:
            ypoints = []
            for i in works[policy]:
                ypoints.append(i)
            ypoints = np.array(ypoints)

            if showErrorBar:
                for w in range(len(xpoints)):
                    d = np.array(TheDataSet[policy][w])
                    l, u = st.norm.interval(confidence=0.95, loc=np.mean(d), scale=st.sem(d))
                    plt.errorbar(x=xpoints[w], y=ypoints[w], yerr=[[np.mean(d)-l], [u-np.mean(d)]], color=colours[policy], capsize=8)

            if int_to_alg[policy]!='DWP' and int_to_alg[policy]!='MAP':
                plt.plot(xpoints, ypoints, linestyle=linestyles[list(linestyles.keys())[5]], linewidth = '2.5', marker=markers[policy], ms='10', markeredgecolor="k", label=int_to_alg[policy], color=colours[policy], alpha=0.5)
            else:
                plt.plot(xpoints, ypoints, linestyle=linestyles[list(linestyles.keys())[5]], linewidth = '2.5', marker=markers[policy], ms='10', markeredgecolor="k", label=int_to_alg[policy], color=colours[policy])

        font = {'family':'serif','color':'darkred','size':12}
        plt.ylabel("Work", fontdict=font)
        plt.xlabel("Weights", fontdict=font)
        plt.title(sys.argv[5])
        # plt.legend(fontsize="26")
        plt.xticks(xpoints) 
        plt.yscale('log')
        plt.grid(axis='y', color='0.80', which='major')
        # plt.savefig(sys.argv[5]+"E4.pdf", format="pdf", bbox_inches="tight")
        plt.show()
    
    ##############################################
    elif sys.argv[2] == '3':

        table_0 = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        count_table_0 = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        TheDataSet_0 = [[[] for _ in range(int(sys.argv[4]))] for _ in range(int(sys.argv[3]))]

        with open("./localJobs/OutputData/"+sys.argv[5]+".txt", "r") as f:
            for line in f:
                data = line.split()
                if(len(data)): ## To check for empy lines
                    if data[0] == "STP" and (data[5] in list(weight_to_int.keys())) and (int(data[3]) in list(int_to_alg.keys())) : #and data[3]!='0' and data[3]!='1' and data[3]!='6' and data[3]!='8': # and data[9]!='0':
                        table_0[int(data[3])][weight_to_int[data[5]]] += float(data[7])
                        count_table_0[int(data[3])][weight_to_int[data[5]]] += 1
                        TheDataSet_0[int(data[3])][weight_to_int[data[5]]].append(float(data[7]))
                        # print(float(data[7]), count_table[int(data[3])][weight_to_int[data[5]]], sep=" ")

        result_0 = np.divide(table_0, count_table_0)

        ##Experiment 3.1: print total Running time/weight table
        table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        count_table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        TheDataSet = [[[] for _ in range(int(sys.argv[4]))] for _ in range(int(sys.argv[3]))]

        with open("./localJobs/OutputData/"+sys.argv[5]+".txt", "r") as f:
            for line in f:
                data = line.split()
                if(len(data)): ## To check for empy lines
                    if data[0] == "STP" and (data[5] in list(weight_to_int.keys())) and (int(data[3]) in list(int_to_alg.keys())) : #and data[3]!='0' and data[3]!='1' and data[3]!='6' and data[3]!='8': # and data[9]!='0':
                        table[int(data[3])][weight_to_int[data[5]]] += float(data[11])
                        count_table[int(data[3])][weight_to_int[data[5]]] += 1
                        TheDataSet[int(data[3])][weight_to_int[data[5]]].append(float(data[11]))
                        # print(float(data[7]), count_table[int(data[3])][weight_to_int[data[5]]], sep=" ")

        result = np.divide(table, count_table)
        thousand = np.ones((int(sys.argv[3]), int(sys.argv[4])))
        thousand = thousand * 1000
        result = np.divide(result, thousand)
        
        if False:
            print()
            print('=======================================  Average RunTime (x 10^6) Table  ========================================')
            print('Alg / Weight|  1.50   |  2.00   |  3.00   |  4.00   |  5.00   |  6.00   |  7.00   |  8.00   |  9.00   |  10.0   |')
            print('_________________' * 7)
            for i in range(len(result)):
                if i!=-1:
                    print(int_to_alg[i],end="")
                    for k in range(12-len(int_to_alg[i])):
                        print(' ',end="")
                    print('|', end="")
                    for j in range(len(result[i])):
                        print(round(result[i][j], 2), end="")
                        for k in range(9-len(str(round(result[i][j], 2)))):
                            print(' ',end="")
                        print('|', end="")
                    print()
            print('__________________' * 7)
                        
            for cnt in range(len(result)):
                print(str(cnt+1)+' place Alg |', end="")
                for i in range(len(result[0])):
                    col = result[:,i]
                    print(int_to_alg[np.argsort(col)[cnt]], end="")
                    # print(int_to_alg[np.argmin(col)], end="")
                    for k in range(9-len(int_to_alg[np.argsort(col)[cnt]])):
                        print(' ',end="")
                    print('|', end="")
                print()
            
            print('=====================================================',end='')
            for _ in range((26 - len(sys.argv[5]))//2):
                print(' ', end='')
            print(sys.argv[5], end='')
            for _ in range((26 - len(sys.argv[5]))//2):
                print(' ', end='')
            print('====================================================')
            print()
        
        ##Experiment 3.2: print Running time Per Node/weight table
        result = np.multiply(result, thousand)
        result = np.divide(result, result_0)
        
        if True:
            print()
            print('=====================================  Average RunTime/Node (x 10^9) Table  ======================================')
            print('Alg / Weight|  1.50   |  2.00   |  3.00   |  4.00   |  5.00   |  6.00   |  7.00   |  8.00   |  9.00   |  10.0   |')
            print('_________________' * 7)
            for i in range(len(result)):
                if i!=-1:
                    print(int_to_alg[i],end="")
                    for k in range(12-len(int_to_alg[i])):
                        print(' ',end="")
                    print('|', end="")
                    for j in range(len(result[i])):
                        print(round(result[i][j], 2), end="")
                        for k in range(9-len(str(round(result[i][j], 2)))):
                            print(' ',end="")
                        print('|', end="")
                    print()
            print('__________________' * 7)
                        
            for cnt in range(len(result)):
                print(str(cnt+1)+' place Alg |', end="")
                for i in range(len(result[0])):
                    col = result[:,i]
                    print(int_to_alg[np.argsort(col)[cnt]], end="")
                    # print(int_to_alg[np.argmin(col)], end="")
                    for k in range(9-len(int_to_alg[np.argsort(col)[cnt]])):
                        print(' ',end="")
                    print('|', end="")
                print()
            
            print('=====================================================',end='')
            for _ in range((26 - len(sys.argv[5]))//2):
                print(' ', end='')
            print(sys.argv[5], end='')
            for _ in range((26 - len(sys.argv[5]))//2):
                print(' ', end='')
            print('====================================================')
            print()

    ##############################################
    elif sys.argv[2] == '4':

        ##Experiment 4: Creates the total Running time/weight plot
        ##arg[3] is the number of algs and arg[4] is the number of weights
        table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        count_table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        TheDataSet = [[[] for _ in range(int(sys.argv[4]))] for _ in range(int(sys.argv[3]))]

        with open("./localJobs/OutputData/"+sys.argv[5]+".txt", "r") as f:
            for line in f:
                data = line.split()
                if(len(data)): ## To check for empy lines
                    if data[0] == "STP" and (data[5] in list(weight_to_int.keys())) and (int(data[3]) in list(int_to_alg.keys())) : #and data[3]!='0' and data[3]!='1' and data[3]!='6' and data[3]!='8': # and data[9]!='0':
                        table[int(data[3])][weight_to_int[data[5]]] += float(data[11])
                        count_table[int(data[3])][weight_to_int[data[5]]] += 1
                        TheDataSet[int(data[3])][weight_to_int[data[5]]].append(float(data[11]))

        result = np.divide(table, count_table)

        thousand = np.ones((int(sys.argv[3]), int(sys.argv[4])))
        thousand = thousand * 1000000000
        result = np.divide(result, thousand)

        Weights = [1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
        
        xpoints = np.array(Weights)
        works = []
        for policy in int_to_alg:
            work_i = []
            for w in list(weight_to_int.keys()):
                work_i.append(result[policy][weight_to_int[w]])
            works.append(work_i)
        for policy in int_to_alg:
            ypoints = []
            for i in works[policy]:
                ypoints.append(i)
            ypoints = np.array(ypoints)

            if showErrorBar:
                for w in range(len(xpoints)):
                    d = np.array(TheDataSet[policy][w])
                    l, u = st.norm.interval(confidence=0.95, loc=np.mean(d), scale=st.sem(d))
                    plt.errorbar(x=xpoints[w], y=ypoints[w], yerr=[[np.mean(d)-l], [u-np.mean(d)]], color=colours[policy], capsize=8)

            if int_to_alg[policy]!='DWP' and int_to_alg[policy]!='MAP':
                plt.plot(xpoints, ypoints, linestyle=linestyles[list(linestyles.keys())[5]], linewidth = '2.5', marker=markers[policy], ms='10', markeredgecolor="k", label=int_to_alg[policy], color=colours[policy], alpha=0.5)
            else:
                plt.plot(xpoints, ypoints, linestyle=linestyles[list(linestyles.keys())[5]], linewidth = '2.5', marker=markers[policy], ms='10', markeredgecolor="k", label=int_to_alg[policy], color=colours[policy])

        font = {'family':'serif','color':'darkred','size':12}
        plt.ylabel("Total RunTime", fontdict=font)
        plt.xlabel("Weights", fontdict=font)
        plt.title(sys.argv[5])
        # plt.legend(fontsize="26")
        plt.xticks(xpoints) 
        plt.yscale('log')
        plt.grid(axis='y', color='0.80', which='major')
        # plt.savefig(sys.argv[5]+"E4.pdf", format="pdf", bbox_inches="tight")
        plt.show()   
    
############################################################################################
elif sys.argv[1] == '-map':
    if sys.argv[2] == '1':

        ##Experiment 1: Creates a table from average of runs of different problems over different weights
        table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        count_table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        TheDataSet = [[[] for _ in range(int(sys.argv[4]))] for _ in range(int(sys.argv[3]))]

        with open("./localJobs/OutputData/"+sys.argv[5]+".txt", "r") as f:
            for line in f:
                data = line.split()
                if(len(data)): ## To check for empy lines
                    if data[0] == "MAP" and (data[7] in list(weight_to_int.keys())) and (int(data[5]) in list(int_to_alg.keys())): #and data[5]!='0' and data[5]!='1' and data[5]!='6' and data[5]!='8':# and data[9]!='0':
                        table[int(data[5])][weight_to_int[data[7]]] += int(data[9])
                        count_table[int(data[5])][weight_to_int[data[7]]] += 1
                        TheDataSet[int(data[5])][weight_to_int[data[7]]].append(int(data[9]))

        result = np.divide(table, count_table)
        print()
        print('==========================================  Average Expansions Table  ===========================================')
        print('Alg / Weight|  1.50   |  2.00   |  3.00   |  4.00   |  5.00   |  6.00   |  7.00   |  8.00   |  9.00   |  10.0   |')
        print('_________________' * 7)
        for i in range(len(table)):
            if i!=-1:
                print(int_to_alg[i],end="")
                for k in range(12-len(int_to_alg[i])):
                    print(' ',end="")
                print('|', end="")
                for j in range(len(table[i])):
                    print(round(result[i][j], 2), end="")
                    for k in range(9-len(str(round(result[i][j], 2)))):
                        print(' ',end="")
                    print('|', end="")
                print()
        print('__________________' * 7)
                    
        for cnt in range(len(table)):
            print(str(cnt+1)+' place Alg |', end="")
            for i in range(len(table[0])):
                col = table[:,i]
                print(int_to_alg[np.argsort(col)[cnt]], end="")
                # print(int_to_alg[np.argmin(col)], end="")
                for k in range(9-len(int_to_alg[np.argsort(col)[cnt]])):
                    print(' ',end="")
                print('|', end="")
            print()
        
        print('=====================================================',end='')
        for _ in range((26 - len(sys.argv[5]))//2):
            print(' ', end='')
        print(sys.argv[5], end='')
        for _ in range((26 - len(sys.argv[5]))//2):
            print(' ', end='')
        print('====================================================')
        print()

    ##############################################
    elif sys.argv[2] == '2':

        ##Experiment 2: Creates the work/weight plot
        ##arg[3] is the number of algs and arg[4] is the number of weights
        table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        count_table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        TheDataSet = [[[] for _ in range(int(sys.argv[4]))] for _ in range(int(sys.argv[3]))]

        with open("./localJobs/OutputData/"+sys.argv[5]+".txt", "r") as f:
            for line in f:
                data = line.split()
                if(len(data)): ## To check for empy lines
                    if data[0] == "MAP" and (data[7] in list(weight_to_int.keys())) and (int(data[5]) in list(int_to_alg.keys())) : #and data[5]!='0' and data[5]!='1' and data[5]!='6' and data[5]!='8':# and data[9]!='0':
                        table[int(data[5])][weight_to_int[data[7]]] += int(data[9])
                        count_table[int(data[5])][weight_to_int[data[7]]] += 1
                        TheDataSet[int(data[5])][weight_to_int[data[7]]].append(int(data[9]))

        result = np.divide(table, count_table)
        Weights = [1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
        xpoints = np.array(Weights)
        works = []
        for policy in int_to_alg:
            work_i = []
            for w in list(weight_to_int.keys()):
                work_i.append(result[policy][weight_to_int[w]])
            works.append(work_i)
        for policy in int_to_alg:
            ypoints = []
            for i in works[policy]:
                ypoints.append(i)
            ypoints = np.array(ypoints)

            if int_to_alg[policy]!='DWP' and int_to_alg[policy]!='MAP':
                plt.plot(xpoints, ypoints, linestyle=linestyles[list(linestyles.keys())[5]], linewidth = '2.5', marker=markers[policy], ms='10', markeredgecolor="k", label=int_to_alg[policy], color=colours[policy], alpha=0.5)
            else:
                plt.plot(xpoints, ypoints, linestyle=linestyles[list(linestyles.keys())[5]], linewidth = '2.5', marker=markers[policy], ms='10', markeredgecolor="k", label=int_to_alg[policy], color=colours[policy])
            
            if showErrorBar:
                for w in range(len(xpoints)):
                    d = np.array(TheDataSet[policy][w])
                    l, u = st.norm.interval(confidence=0.95, loc=np.mean(d), scale=st.sem(d))
                    plt.errorbar(x=xpoints[w], y=ypoints[w], yerr=[[np.mean(d)-l], [u-np.mean(d)]], color=colours[policy], capsize=8)
            
        font = {'family':'serif','color':'darkred','size':12}
        plt.ylabel("Work", fontdict=font)
        plt.xlabel("Weights", fontdict=font)
        plt.title(sys.argv[5])
        # plt.legend(fontsize="25")
        plt.xticks(xpoints) 
        plt.yscale('log')
        # plt.xscale('log')
        plt.grid(axis='y', color='0.80', which='major')
        # plt.savefig(sys.argv[5]+"E4.pdf", format="pdf", bbox_inches="tight")
        plt.show()
    
    ##############################################
    elif sys.argv[2] == '3':

        table_0 = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        count_table_0 = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        TheDataSet_0 = [[[] for _ in range(int(sys.argv[4]))] for _ in range(int(sys.argv[3]))]

        with open("./localJobs/OutputData/"+sys.argv[5]+".txt", "r") as f:
            for line in f:
                data = line.split()
                if(len(data)): ## To check for empy lines
                    if data[0] == "MAP" and (data[7] in list(weight_to_int.keys())) and (int(data[5]) in list(int_to_alg.keys())): #and data[5]!='0' and data[5]!='1' and data[5]!='6' and data[5]!='8':# and data[9]!='0':
                        table_0[int(data[5])][weight_to_int[data[7]]] += int(data[9])
                        count_table_0[int(data[5])][weight_to_int[data[7]]] += 1
                        TheDataSet_0[int(data[5])][weight_to_int[data[7]]].append(int(data[9]))

        result_0 = np.divide(table_0, count_table_0)

        ##Experiment 3.1: print total Running time/weight table
        table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        count_table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        TheDataSet = [[[] for _ in range(int(sys.argv[4]))] for _ in range(int(sys.argv[3]))]

        with open("./localJobs/OutputData/"+sys.argv[5]+".txt", "r") as f:
            for line in f:
                data = line.split()
                if(len(data)): ## To check for empy lines
                    if data[0] == "MAP" and (data[7] in list(weight_to_int.keys())) and (int(data[5]) in list(int_to_alg.keys())): #and data[5]!='0' and data[5]!='1' and data[5]!='6' and data[5]!='8':# and data[9]!='0':
                        table[int(data[5])][weight_to_int[data[7]]] += float(data[11])
                        count_table[int(data[5])][weight_to_int[data[7]]] += 1
                        TheDataSet[int(data[5])][weight_to_int[data[7]]].append(float(data[11]))

        result = np.divide(table, count_table)
        thousand = np.ones((int(sys.argv[3]), int(sys.argv[4])))
        thousand = thousand * 1000
        result = np.divide(result, thousand)
        
        if False:
            print()
            print('=======================================  Average RunTime (x 10^6) Table  ========================================')
            print('Alg / Weight|  1.50   |  2.00   |  3.00   |  4.00   |  5.00   |  6.00   |  7.00   |  8.00   |  9.00   |  10.0   |')
            print('_________________' * 7)
            for i in range(len(result)):
                if i!=-1:
                    print(int_to_alg[i],end="")
                    for k in range(12-len(int_to_alg[i])):
                        print(' ',end="")
                    print('|', end="")
                    for j in range(len(result[i])):
                        print(round(result[i][j], 2), end="")
                        for k in range(9-len(str(round(result[i][j], 2)))):
                            print(' ',end="")
                        print('|', end="")
                    print()
            print('__________________' * 7)
                        
            for cnt in range(len(result)):
                print(str(cnt+1)+' place Alg |', end="")
                for i in range(len(result[0])):
                    col = result[:,i]
                    print(int_to_alg[np.argsort(col)[cnt]], end="")
                    # print(int_to_alg[np.argmin(col)], end="")
                    for k in range(9-len(int_to_alg[np.argsort(col)[cnt]])):
                        print(' ',end="")
                    print('|', end="")
                print()
            
            print('=====================================================',end='')
            for _ in range((26 - len(sys.argv[5]))//2):
                print(' ', end='')
            print(sys.argv[5], end='')
            for _ in range((26 - len(sys.argv[5]))//2):
                print(' ', end='')
            print('====================================================')
            print()
        
        ##Experiment 3.2: print Running time Per Node/weight table
        result = np.multiply(result, thousand)
        result = np.divide(result, result_0)
        
        if True:
            print()
            print('=====================================  Average RunTime/Node (x 10^9) Table  ======================================')
            print('Alg / Weight|  1.50   |  2.00   |  3.00   |  4.00   |  5.00   |  6.00   |  7.00   |  8.00   |  9.00   |  10.0   |')
            print('_________________' * 7)
            for i in range(len(result)):
                if i!=-1:
                    print(int_to_alg[i],end="")
                    for k in range(12-len(int_to_alg[i])):
                        print(' ',end="")
                    print('|', end="")
                    for j in range(len(result[i])):
                        print(round(result[i][j], 2), end="")
                        for k in range(9-len(str(round(result[i][j], 2)))):
                            print(' ',end="")
                        print('|', end="")
                    print()
            print('__________________' * 7)
                        
            for cnt in range(len(result)):
                print(str(cnt+1)+' place Alg |', end="")
                for i in range(len(result[0])):
                    col = result[:,i]
                    print(int_to_alg[np.argsort(col)[cnt]], end="")
                    # print(int_to_alg[np.argmin(col)], end="")
                    for k in range(9-len(int_to_alg[np.argsort(col)[cnt]])):
                        print(' ',end="")
                    print('|', end="")
                print()
            
            print('=====================================================',end='')
            for _ in range((26 - len(sys.argv[5]))//2):
                print(' ', end='')
            print(sys.argv[5], end='')
            for _ in range((26 - len(sys.argv[5]))//2):
                print(' ', end='')
            print('====================================================')
            print()
        
    ##############################################
    elif sys.argv[2] == '4':

        ##Experiment 4: Creates the total Running time/weight plot
        ##arg[3] is the number of algs and arg[4] is the number of weights
        table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        count_table = np.zeros((int(sys.argv[3]), int(sys.argv[4])))
        TheDataSet = [[[] for _ in range(int(sys.argv[4]))] for _ in range(int(sys.argv[3]))]

        with open("./localJobs/OutputData/"+sys.argv[5]+".txt", "r") as f:
            for line in f:
                data = line.split()
                if(len(data)): ## To check for empy lines
                    if data[0] == "MAP" and (data[7] in list(weight_to_int.keys())) and (int(data[5]) in list(int_to_alg.keys())) : #and data[5]!='0' and data[5]!='1' and data[5]!='6' and data[5]!='8':# and data[9]!='0':
                        table[int(data[5])][weight_to_int[data[7]]] += float(data[11])
                        count_table[int(data[5])][weight_to_int[data[7]]] += 1
                        TheDataSet[int(data[5])][weight_to_int[data[7]]].append(float(data[11]))

        result = np.divide(table, count_table)

        thousand = np.ones((int(sys.argv[3]), int(sys.argv[4])))
        thousand = thousand * 1000000000
        result = np.divide(result, thousand)

        Weights = [1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
        xpoints = np.array(Weights)
        works = []
        for policy in int_to_alg:
            work_i = []
            for w in list(weight_to_int.keys()):
                work_i.append(result[policy][weight_to_int[w]])
            works.append(work_i)
        for policy in int_to_alg:
            ypoints = []
            for i in works[policy]:
                ypoints.append(i)
            ypoints = np.array(ypoints)

            if int_to_alg[policy]!='DWP' and int_to_alg[policy]!='MAP':
                plt.plot(xpoints, ypoints, linestyle=linestyles[list(linestyles.keys())[5]], linewidth = '2.5', marker=markers[policy], ms='10', markeredgecolor="k", label=int_to_alg[policy], color=colours[policy], alpha=0.5)
            else:
                plt.plot(xpoints, ypoints, linestyle=linestyles[list(linestyles.keys())[5]], linewidth = '2.5', marker=markers[policy], ms='10', markeredgecolor="k", label=int_to_alg[policy], color=colours[policy])
            
            if showErrorBar:
                for w in range(len(xpoints)):
                    d = np.array(TheDataSet[policy][w])
                    l, u = st.norm.interval(confidence=0.95, loc=np.mean(d), scale=st.sem(d))
                    plt.errorbar(x=xpoints[w], y=ypoints[w], yerr=[[np.mean(d)-l], [u-np.mean(d)]], color=colours[policy], capsize=8)
            
        font = {'family':'serif','color':'darkred','size':12}
        plt.ylabel("Total RunTime", fontdict=font)
        plt.xlabel("Weights", fontdict=font)
        plt.title(sys.argv[5])
        # plt.legend(fontsize="25")
        plt.xticks(xpoints) 
        plt.yscale('log')
        # plt.xscale('log')
        plt.grid(axis='y', color='0.80', which='major')
        # plt.savefig(sys.argv[5]+"E4.pdf", format="pdf", bbox_inches="tight")
        plt.show()
    

    