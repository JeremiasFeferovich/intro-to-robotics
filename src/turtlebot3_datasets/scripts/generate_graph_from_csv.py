#!/usr/bin/env python3

import csv
import numpy as np
import matplotlib.pyplot as plt
import argparse


def read_csv(file_name):
    time_values = []
    error_x = []
    error_y = []
    yaw_errors = []

    with open(file_name, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row

        for row in reader:
            time_values.append(float(row[0]))
            error_x.append(float(row[1]))
            error_y.append(float(row[2]))
            yaw_errors.append(float(row[3]))

    return np.array(time_values), np.array(error_x), np.array(error_y), np.array(yaw_errors)


def plot_error_graphs(time_values, error_x, error_y, yaw_errors, output_prefix):
    sum_abs_xy_errors = np.abs(error_x) + np.abs(error_y)

    plt.figure()
    plt.plot(time_values, np.abs(error_x), label='|Error X| (mm)')
    plt.plot(time_values, np.abs(error_y), label='|Error Y| (mm)')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Error (mm)')
    plt.title('Absolute Translation Errors (X, Y) over Time')
    plt.legend()
    plt.grid(True)
    plt.savefig(f'{output_prefix}_abs_translation_errors.png')
    plt.close()

    plt.figure()
    plt.plot(time_values, sum_abs_xy_errors,
             label='Sum of |Error X| and |Error Y| (mm)')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Error (mm)')
    plt.title('Sum of Absolute X and Y Errors over Time')
    plt.legend()
    plt.grid(True)
    plt.savefig(f'{output_prefix}_sum_abs_xy_errors.png')
    plt.close()

    plt.figure()
    plt.plot(time_values, np.abs(yaw_errors), label='|Yaw| (rad)')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Yaw (radians)')
    plt.title('Absolute Yaw Error over Time')
    plt.legend()
    plt.grid(True)
    plt.savefig(f'{output_prefix}_abs_yaw_error.png')
    plt.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate error graphs (X, Y, Yaw) from CSV file.")
    parser.add_argument(
        'csv_file', help="Path to the CSV file containing error data")
    parser.add_argument('--output_prefix', default='error_plot_from_csv',
                        help="Prefix for saving the output images (default: error_plot_from_csv)")

    args = parser.parse_args()

    csv_file = args.csv_file
    output_prefix = args.output_prefix

    time_values, error_x, error_y, yaw_errors = read_csv(csv_file)

    plot_error_graphs(time_values, error_x, error_y, yaw_errors, output_prefix)
