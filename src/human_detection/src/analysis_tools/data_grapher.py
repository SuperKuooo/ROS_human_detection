import matplotlib.pyplot as plt
import numpy as np
import sys
import random


def graph_run_time(list_of_run_time):
    f1 = plt.figure(1)
    list_of_run_time = data_reduce(list_of_run_time)

    average_run_time = sum(list_of_run_time)/len(list_of_run_time)
    x_range = range(0, len(list_of_run_time))
    plt.scatter(x_range, list_of_run_time, label='Raw Run Time')
    plt.axhline(y=average_run_time, color='r', label='Average Run Time')

    plt.title('Run Time of Each Frame on Ryzen 5 CPU 3600 Hz')
    plt.xlabel('Number of Frame')
    plt.ylabel('Processing time (Seconds)')
    plt.legend()
    # plt.show()


def graph_average_percentage(list_of_percentage, pass_line = 0.6):
    # list_of_percentage = data_reduce(list_of_percentage)
    f2 = plt.figure(2)

    average_percentage = sum(list_of_percentage)/len(list_of_percentage)
    x_range = range(0, len(list_of_percentage))
    plt.scatter(x_range, list_of_percentage, label='Raw Percentage')
    plt.axhline(y=average_percentage, color='r',
                label='Average Percentage')
    plt.axhline(y=pass_line, color='g',
                label='Passing Score')

    plt.title('Confidence Level of Each Frame')
    plt.xlabel('Number of Frame')
    plt.ylabel('Confidence Level (Percentage %)')
    plt.legend()
    plt.show()


def data_reduce(data_list, output_len=60):
    if len(data_list) > output_len:
        return_list = []
        for _ in range(0, output_len):
            x = random.randint(0, len(data_list) - 1)
            return_list.append(data_list[x])
            del data_list[x]
        return return_list
    return data_list


if __name__ == '__main__':
    temp = []
    for i in range(0, 50):
        temp.append(random.random())
    graph_run_time(temp)
