def main():
    inp = " "
    while input("Do you want to continue? [y/n]: ") != "n":
        x1 = int(input("x1: "))
        x2 = int(input("x2: "))
        print(f"Relative error: {(100 * relative_error(x1, x2)):.2f}% \n")

def relative_error(x1, x2):
        return abs(x1 - x2) / (abs(x1+x2))


if __name__ == '__main__':
    main()

