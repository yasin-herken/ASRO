import hungarian
import numpy

if __name__ == "__main__":
    mtrx = numpy.array(
        [
            [2500, 4000, 3500],
            [4000, 6000, 3500],
            [2000, 4000, 2500],    
        ]
    )
    hungarian = hungarian.Hungarian()
    hungarian.calculate(mtrx)

    for i in hungarian.get_results():
        print(i)
