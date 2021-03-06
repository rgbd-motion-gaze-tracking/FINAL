'''
Debugging and introspection utilities.

This file is part of the Robotic Teleoperation via Motion and Gaze Tracking
MQP from the Worcester Polytechnic Institute.

Written in 2021 by Nicholas Hollander <nhhollander@wpi.edu>
'''

import time
import atexit

FUNCPERF_ENABLE = True
FUNCPERF_WRITETOFILE = True
funcperf_data = dict()

def funcperf(func):
    '''
    Measure Function Performance.

    This function decorator records the run time of the given function assuming
    `FUNCPERF_ENABLE` is set to `True`.
    '''
    def wrapper(*args, **kwargs):
        if not FUNCPERF_ENABLE:
            return func(*args, **kwargs)
        name = f"{func.__module__}.{func.__name__}"
        start = time.time()

        result = func(*args, **kwargs)

        if not name in funcperf_data:
            funcperf_data[name] = [0, 0]
        funcperf_data[name][0] += time.time() - start
        funcperf_data[name][1] += 1

        return result
    return wrapper

def funcperf_report():
    '''
    Generate a function performance report.

    This method takes the data collected
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
    asarray = [
        (a, funcperf_data[a][0], funcperf_data[a][1], funcperf_data[a][0]/funcperf_data[a][1])
        for a in funcperf_data]
    for s in sorted(asarray, key=lambda a: a[3], reverse=True):
        line(
            s[0],
            pretty_time(s[1]),
            str(s[2]),
            pretty_time(s[3])
        )
    line('-'*50, '-'*50, '-'*50, '-'*50) # Adds lines
    # Write report to file
    if FUNCPERF_WRITETOFILE:
        handle = open("/tmp/funcperf.csv", "w")
        for a in funcperf_data:
            handle.write(
                f"{a},"
                f"{funcperf_data[a][0]},"
                f"{funcperf_data[a][1]},"
                f"{funcperf_data[a][0]/funcperf_data[a][1]}\n")
        handle.close()

# Register the performance output
atexit.register(funcperf_report)
