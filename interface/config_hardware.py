class ServoParams:
    def __init__(self):
        self.pins = [
            [1, 5, 9],
            [2, 6, 10],
            [3, 7, 11],
            [4, 8, 12],
        ]

        self.directions = [[0, 0, 0] * 4]

        self.rangePWMs = [
            [2000, 3000],
            [2000, 3000],
            [2000, 3000],
            [2000, 3000],
        ]

        self.rangeAngles = [
            [0, 100],
            [0, 100],
            [0, 100],
            [0, 100],
        ]
