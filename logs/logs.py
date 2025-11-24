"""
Conversion of log files to CSV format
"""

### imports ###
import csv

### functions ###
def readFile(filename):
    file = open(filename, "r")

    throttle1Values = ['Throttle1']
    throttle2Values = ['Throttle2']
    TorqueValues = ['Torque Requested']
    TimestampValues = ['Timestamp']

    num_lines = file.readlines()
    for line in num_lines:
        if "Throttle1:" in line:
            throttle1Values.append(float(line.strip().split(": ")[1]))
        if "Throttle2:" in line:
            throttle2Values.append(float(line.strip().split(": ")[1]))
        if "Torque Requested:" in line:
            TorqueValues.append(float(line.strip().split(": ")[1]))
            TimestampValues.append((line.strip().split(" ")[0]))

    file.close()

    return throttle1Values, throttle2Values, TorqueValues, TimestampValues

def main ():
    throttle1, throttle2, torque, timestamp = readFile("test.log")

    with open('data.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        for values in zip(throttle1, throttle2, torque, timestamp):
            writer.writerow(values)
    
    print("CSV files created successfully.")

### call main ###
if __name__ == "__main__":
    main()