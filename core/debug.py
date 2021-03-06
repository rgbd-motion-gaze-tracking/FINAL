##
# Debugging and introspection utilities
#
# This file is part of the Robotic Teleoperation via Motion and Gaze Tracking
# MQP from the Worcester Polytechinc Institute.
#
# Copyright 2021 Nicholas Hollander <nhhollander@wpi.edu>
#

import time
import atexit

funcperf_enable = True
funcperf_writetofile = True
funcperf_data = dict()

##
# Measure Function Performance
#
# This function decorator measures the performance of the given function
# assuming `funcperf_enable` is set to true.
def funcperf(func):
    def wrapper(*args, **kwargs):
        if not funcperf_enable:
            return func(*args, **kwargs)
        
        name = f"{func.__module__}.{func.__name__}"
        start = time.time()

        result = func(*args, **kwargs)

        # Store duration
        if not name in funcperf_data:
            funcperf_data[name] = [0,0]
        funcperf_data[name][0] += time.time() - start
        funcperf_data[name][1] += 1

        return result
    return wrapper

def funcperf_report():
    '''
    Generate a function performance report. This method takes the data collected
    by the `funcperf` decorator and prints it out as a nice table. If enabled,
    it also generates a CSV report.
    '''
    def line(a, b, c, d):
        len_a = 50
        len_b = 10
        len_c = 10
        len_d = 10
        a = (a + ' '*len_a)[:len_a]
        b = (' '*len_b + b)[-len_b:]
        c = (' '*len_c + c)[-len_c:]
        d = (' '*len_d + d)[-len_d:]
        print(f"|{a}|{b}|{c}|{d}|")
    def pretty_time(t):
        suffix = "??"
        if t < (1/1000000): # Nanoseconds
            t *= 1000000000
            suffix = "ns"
        elif t < (1/1000): # Microsecond
            t *= 1000000
            suffix = "μs"
        elif t < 1: # Millisecond
            t *= 1000
            suffix = "ms"
        elif t < 60*5: # Second
            suffix = "s "
        elif t < 60*60: # Minute
            t /= 60
            suffix = "m "
        else: # hour
            t /= 60*60
            suffix = "h "
        return f"{t:>6.2f} {suffix}"
    print("\033[1mPerformance Report:\033[0m")
    line("Function", "Tot. Exec", "Calls", "Avg. Exec")
    line('-'*50, '-'*50, '-'*50, '-'*50) # Adds lines
    asarray = [(a, funcperf_data[a][0], funcperf_data[a][1], funcperf_data[a][0]/funcperf_data[a][1]) for a in funcperf_data]
    for s in sorted(asarray, key=lambda a: a[3], reverse=True):
        line(
            s[0],
            pretty_time(s[1]),
            str(s[2]),
            pretty_time(s[3])
        )
    line('-'*50, '-'*50, '-'*50, '-'*50) # Adds lines
    # Write report to file
    if funcperf_writetofile:
        handle = open("/tmp/funcperf.csv", "w")
        for a in funcperf_data:
            handle.write(f"{a},{funcperf_data[a][0]},{funcperf_data[a][1]},{funcperf_data[a][0]/funcperf_data[a][1]}\n")
        handle.close()

# Register the performance output
atexit.register(funcperf_report)
