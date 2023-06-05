# Manually Operated Car
# periods = [10,50,50,500,10,200]
# execTimes = [0.23,0.06,0.07,55.10,1.2,60.08]

# Autonomous Car
periods = [100,50,50,500,10,50,10]
execTimes = [58.78,0.09,0.06,61.76,1.21,0.60,0.30]

def rwc(task):
    previous_rwc = -1
    current_rwc = sum(execTimes)
    while previous_rwc != current_rwc:
        previous_rwc = current_rwc
        current_rwc = execTimes[task]
        for i in range(len(periods)):
            if i != task:
                current_rwc += previous_rwc * execTimes[i] / periods[i]
    return current_rwc

for i in range(len(periods)):
    print(f"task{i}: Rwc = {rwc(i)} ms - Deadline = {periods[i]} ms")