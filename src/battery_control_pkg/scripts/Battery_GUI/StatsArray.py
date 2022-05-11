
import numpy as np
import math



class StatsArray:
    """
    This class takes an array and returns the following statistics:
    - Mean
    - Median
    - Standard Deviation
    - Variance
    - Minimum
    - Maximum
    - Range
    - Skewness
    - Kurtosis
    - Percentile 25
    - Percentile 50
    - Percentile 75

    """

    def __init__(self, array):
        self.array = array
        self.mean = self.meanarray(self.array)
        self.median = self.medianarray(self.array)
        self.var = self.vararray(self.array)
        # standard error is the first element of the tuple
        self.stderr = self.stderrarray(self.array)[0]
        self.min = self.minarray(self.array)
        self.max = self.maxarray(self.array)
        self.range = self.max - self.min

    def skewnessarray(self, arr):
        """
        This function takes an array and returns the skewness value in the array.
        """
        median = self.median
        mean = self.mean
        stderr = self.stderr

        skewnes1 = 3*(median-mean)/stderr
        return skewnes1

    def skewnessarray2(self, arr):
        """
        This function takes an array and returns the skewness value in the array.
        """
        size = len(arr)
        mean = self.mean
        stderr = self.stderr
        total_err = 0
        for i in range(0, len(arr)):
            total_err = total_err + (arr[i] - mean)**3
        skewness2 = total_err / (size*(stderr**3))
        return skewness2

    def kurtosisarray(self, arr):
        """
        This function takes an array and returns the kurtosis value in the array.
        """
        size = len(arr)
        mean = self.mean
        stderr = self.stderr
        total_err = 0
        for i in range(0, len(arr)):
            total_err = total_err + (arr[i] - mean)**4

        kurtosis = total_err / (size*(stderr**4))
        return kurtosis

    def maxarray(self, arr):
        """
        This function takes an array and returns the maximum value in the array.
        """
        max_value = arr[0]
        for i in range(1, len(arr)):
            if arr[i] > max_value:
                max_value = arr[i]
        return max_value

    def minarray(self, arr):
        """
        This function takes an array and returns the minimum value in the array.
        """
        min_value = arr[0]
        for i in range(1, len(arr)):
            if arr[i] < min_value:
                min_value = arr[i]
        return min_value

    def sumarray(self, arr):
        """
        This function takes an array and returns the sum of all the values in the array.
        """
        sum = 0
        for i in range(0, len(arr)):
            sum = sum + arr[i]
        return sum

    def meanarray(self, arr):
        """
        This function takes an array and returns the mean value in the array.
        """
        sum = 0
        for i in range(0, len(arr)):
            sum = sum + arr[i]
        mean = sum / len(arr)
        return mean

    def medianarray(self, arr):
        """
        This function takes an array and returns the median value in the array.
        """
        arr.sort()
        if len(arr) % 2 == 0:
            median = (arr[int(len(arr) / 2)] + arr[int(len(arr) / 2 - 1)]) / 2
        else:
            median = arr[int(len(arr) / 2)]
        return median

    def vararray(self, arr):
        """
        This function takes an array and returns the standard deviation value in the array.
        """
        sum = 0  # sum of the squares
        for i in range(0, len(arr)):
            sum = sum + arr[i]
        mean = sum / len(arr)
        total_error = 0
        for i in range(0, len(arr)):
            total_error = total_error + (arr[i] - mean)**2
        var = total_error / len(arr)
        return var

    def stderrarray(self, arr):
        """
        This function takes an array and returns the standard error value in the array.
        """
        sum = 0  # sum of the squares
        for i in range(0, len(arr)):
            sum = sum + arr[i]
        mean = sum / len(arr)
        total_error = 0
        for i in range(0, len(arr)):
            total_error = total_error + (arr[i] - mean)**2
        var = total_error / len(arr)
        stderr = var**0.5
        return stderr, var


if __name__ == "__main__":
    # generate a random array of 500 integers between 0 and 1000
    arr = np.random.randint(0, 1000, 500)
    # create a StatsArray object
    stats = StatsArray(arr)
    # print the statistics
    print("Mean:", stats.mean)
    print("Median:", stats.median)
    print("Standard Deviation:", stats.stderr)
    print("Variance:", stats.var)
    print("Minimum:", stats.min)
    print("Maximum:", stats.max)
    print("Range:", stats.range)
    print("Skewness1:", stats.skewnessarray(arr))
    print("Skewness2:", stats.skewnessarray2(arr))
    print("Kurtosis:", stats.kurtosisarray(arr))
