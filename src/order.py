'''Class to represent orders given by customers to waiter bot'''

from collections import namedtuple


def class Order(namedtuple):
    
    def __init__(self, name, food=None, drink=None):
        self.name = name
        self.food = food
        self.drink = drink
    

