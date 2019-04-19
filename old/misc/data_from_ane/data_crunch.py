import csv

# Prints the approximate difference between data points,
# which can be considered rough estmates of on time or off time. Multiply by
# two to get the approximate signal period the datapoint represents, and
# invert that to get the frequency.
diffs = []
with open('becton_blow.csv', 'rb') as csvfile:
    spamreader = csv.reader(csvfile)
    last = 0
    for row in spamreader:
        try:
            data = float(row[0])
        except ValueError:
            continue
        diff = data - last
        last = data
        diffs.append(diff)

# Low pass filter
pruned = []
for thing in diffs:
    if thing < .002: # This is equivalent to a 500Hz change
        pruned.append(1) # Dummy variable
    else:
        pruned.append(thing)
#print diffs.index(0.11926400000000001)
print sorted(pruned) 
