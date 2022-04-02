def press_enter():
    input("\nPress Enter: ")


def yn_question(text):
    positive = ['y', 'Y']
    negative = ['n', 'N']
    while True:
        answer = input(text + " (y/n): ")
        if answer in positive:
            return True
        elif answer in negative:
            return False
        print("Type only 'y', Y' or 'n', 'N'")