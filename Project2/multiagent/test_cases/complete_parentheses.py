##  GIVEN EXAMPLE
##  1 + 2 ) * 3 – 4 ) * 5 – 6 ) ) )
##  ( ( 1 + 2 ) * ( ( 3 – 4 ) * ( 5 – 6 ) ) )
##

## Assumptions we have made
## 1. The structure will be: number1 operator number2
## 2. We do not have white spaces
##

# The idea is that we add the items in a stack until we find the right parenthesis
# Then we pop the last 3 items, since we have the structure number1 operator number2,
# and after that we combine them as one expression, and we sandwich it in closed parenthesis

def  complete_parentheses (list1):
    stack = []  # We use stack implementation which is using a list

    for i in list1:
        if i == ")":     # When we find the right parenthesis we pop the elements and add the left parenthesis
            #  We pop up the last three items in order to add the left parenthesis
            number2 = stack.pop()
            operator = stack.pop()
            number1 = stack.pop()

            # We combine the three items in one expression, so it occupies one slot off the stack
            expression = f"({number1}{operator}{number2})"
            stack.append(expression)    # We add the expression to the stack

        else :  # This means it is either number or operator so we add it to the stack
            stack.append(i)

    # the whole list is now parsed and in the correct form so we return it
    return stack

# Given example
list1 = ["1","+", "2", ")", "*", "3", "–", "4", ")", "*", "5", "–", "6", ")", ")", ")"]
print("Given example:", "".join(list1))
result = complete_parentheses(list1)
print("Completed result:", "".join(result),"\n")

# Example with number greater than 9
list1 = ["12","*", "15", ")", "/", "70", "–", "34", ")", "*", "89",")"]
print("Given example:", "".join(list1))
result = complete_parentheses(list1)
print("Completed result:", "".join(result), "\n")

