import marimo

app = marimo.App()


@app.cell(hide_code=True)
def _(mo):
    mo.md(
        r"""
        # Introduction to Programming with Python
        ---
        """
    )
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(
        """
        **Goals**

        1. Learn the fundamentals of *Python* programming;
        1. Become familiar with *Marimo Notebook*;
        1. Utilize scientific computing modules;
        1. Explore object-oriented programming.
        """
    )
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""## Numerical variables & types""")
    return


@app.cell
def _():
    a = 1 # An integer
    print('The variable a = {} is of type {}'.format(a, type(a)))
    return (a,)


@app.cell
def _():
    b = -1.25 # A floating number
    print('The variable b = {} is of type {}'.format(b, type(b)))
    return (b,)


@app.cell
def _():
    c = 1+0.5j # A complex number 
    print('The variable c = {} is of type {}'.format(c, type(c)))
    return (c,)


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""## Strings""")
    return


@app.cell
def _():
    msg = "My 1st lab!"
    print(msg, type(msg), sep = '\n***\n') # \n: Carriage Return & Line Feed
    print(msg + 3* '\nPython is awesome')
    return (msg,)


@app.cell
def _():
    longMsg = """This is a long message,
    spanned over multiple lines"""
    print(longMsg)
    return (longMsg,)


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""_Indexing and slicing_""")
    return


@app.cell
def _(msg):
    # Positive indexing
    print(msg, msg[1:5], sep = ' -----> ')
    # Negative indexing
    print(msg, msg[-5:-1], sep = ' -----> ')
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""_String transformations_""")
    return


@app.cell
def _():
    msg_1 = 'A message'
    print(len(msg_1))
    print(msg_1.lower())
    print(msg_1.upper())
    print(msg_1.split(' '))
    print(msg_1.replace('mes', 'MES'))
    print('a' in msg_1)
    return (msg_1,)


@app.cell
def _():
    price, number, perso = 300, 7, 'A customer'
    print('{} asks for {} pieces. They cost {} TND!'.format(perso, number, price))
    print('{1} demande {2} pièces. They cost {0} TND!'.format(price, perso, number))
    return number, perso, price


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""## Binary, octal & hexadecimal""")
    return


@app.cell
def _():
    x = 0b0101 # 0b : binary
    print(x, type(x), sep = '\t----\t') # \t : tabular
    y = 0xAF # Ox : hexadecimal
    print(y, type(y), sep = '\t' + '---'*5 + '\t')
    z = 0o010 # 0o : octal
    print(z, type(z), sep = ', ')
    return x, y, z


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""_Boolean_""")
    return


@app.cell
def _():
    a_1 = True
    b_1 = False
    print(a_1)
    print(b_1)
    return a_1, b_1


@app.cell
def _():
    print("50 > 20 ? : {} \n50 < 20 ? : {} \n50 = 20 ? : {}\n50 /= 20 ? : {}"
          .format(50 > 20, 50 < 20, 50 == 20, 50 != 20)
         )
    return


@app.cell
def _():
    print(bool(123), bool(0), bool('Lab'), bool())
    return


@app.cell
def _():
    var1 = 100
    print(isinstance(var1, int))
    var2 = -100.35
    print(isinstance(var2, int))
    print(isinstance(var2, float))
    return var1, var2


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""## Lists, tuples & dictionaries""")
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""In Python, a list is an ordered collection of items that can be of any data type (including other lists). Lists are defined using square brackets, with items separated by commas. For example:""")
    return


@app.cell
def _():
    shopping_list = ['milk', 'eggs', 'bread', 'apples']
    return (shopping_list,)


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""A tuple is also an ordered collection of items, but it is immutable, meaning that the items it contains cannot be modified once the tuple is created. Tuples are defined using parentheses, with items separated by commas. For example:""")
    return


@app.cell
def _():
    point = (3, 5)
    return (point,)


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""A dictionary is a collection of key-value pairs, where the keys are unique and used to look up the corresponding values. Dictionaries are defined using curly braces, with the key-value pairs separated by commas. The keys and values are separated by a colon. For example:""")
    return


@app.cell
def _():
    phonebook = {'Alice': '555-1234', 'Bob': '555-5678', 'Eve': '555-9101'}
    return (phonebook,)


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""You can access the items in a list or tuple using an index, and you can access the values in a dictionary using the corresponding keys. For example:""")
    return


@app.cell
def _(phonebook, point, shopping_list):
    # Accessing the second item in a list
    print(shopping_list[1])  # prints 'eggs'

    # Accessing the first item in a tuple
    print(point[0])  # prints 3

    # Accessing the phone number for 'Bob' in the phonebook dictionary
    print(phonebook['Bob'])  # prints '555-5678'
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""### List""")
    return


@app.cell
def _():
    lst = ['a', 'b', 'c', 1, True] # An aggregate of various types
    print(lst)
    return (lst,)


@app.cell
def _(lst):
    print(len(lst)) # Length of `lst` variable
    print(lst[1:3]) # Accessing elements of `lst`
    lst[0] = ['1', 0] # Combined list
    print(lst)
    print(lst[3:])
    print(lst[:3])
    return


@app.cell
def _(lst):
    lst.append('etc') # Insert 'etc' at the end
    print(lst)
    return


@app.cell
def _(lst):
    lst.insert(1, 'xyz') # Inserting 'xyz'
    print(lst)
    return


@app.cell
def _(lst):
    lst.pop(1)
    print(lst)
    return


@app.cell
def _(lst):
    lst.pop()
    print(lst)
    return


@app.cell
def _(lst):
    del lst[0]
    print(lst)
    return


@app.cell
def _(lst):
    lst.append('b')
    print(lst)
    lst.remove('b')
    print(lst)
    return


@app.cell
def _(lst):
    # Loop
    for k in lst:
        print(k)
    return (k,)


@app.cell
def _(lst):
    lst.clear()
    print(lst)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(
        r"""
        | **_Method_**  | **_Description_** |
        | ------------- | ------------- |
        | **copy()**    | Returns a copy of the list |
        | **list()**    | Transforms into a list |
        | **extend ()** | Extends a list by adding elements at its end |
        | **count()**   | Returns the occurrences of the specified value |
        | **index()**   | Returns the index of the first occurrence of a specified value |
        | **reverse()** | Reverse a list |
        | **sort()**    | Sort a list |
        """
    )
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""### Tuples""")
    return


@app.cell
def _():
    tpl = (1, 2, 3)
    print(tpl)
    return (tpl,)


@app.cell
def _():
    tpl_1 = (1, '1', 2, 'text')
    print(tpl_1)
    return (tpl_1,)


@app.cell
def _(tpl_1):
    print(len(tpl_1))
    return


@app.cell
def _(tpl_1):
    print(tpl_1[1:])
    return


@app.cell
def _(tpl_1):
    try:
        tpl_1.append('xyz')
    except Exception as err:
        print(err)
    return


@app.cell
def _(tpl_1):
    try:
        tpl_1.insert(1, 'xyz')
    except Exception as err:
        print(err)
    return


@app.cell
def _(tpl_1):
    my_lst = list(tpl_1)
    my_lst.append('xyz')
    print(my_lst, type(my_lst), sep=', ')
    return (my_lst,)


@app.cell
def _(my_lst):
    nv_tpl = tuple(my_lst) # Convert 'my_lst' into a tuple 'nv_tpl'
    print(nv_tpl, type(nv_tpl), sep = ', ')
    return (nv_tpl,)


@app.cell
def _(nv_tpl):
    for k_1 in nv_tpl:
        print(k_1)
    return (k_1,)


@app.cell
def _(nv_tpl, tpl_1):
    rs_tpl = tpl_1 + nv_tpl
    print(rs_tpl)
    return (rs_tpl,)


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""### Dictionaries""")
    return


@app.cell
def _():
    # dct = {"key": "value"}
    dct = {
        "Term" : "GM",
        "Speciality" : "ElnI",
        "Sem" : "4"
    }
    print(dct, type(dct), sep = ', ')
    return (dct,)


@app.cell
def _(dct):
    print(dct["Sem"])
    sem = dct.get("Sem")
    print(sem)
    return (sem,)


@app.cell
def _(dct):
    dct["Term"] = "GE"
    print(dct)
    return


@app.cell
def _(dct):
    # Loop
    for el in dct:
        print(el, dct[el], sep = '\t|\t')
    return (el,)


@app.cell
def _(dct):
    for k_2 in dct.keys():
        print(k_2)
    return (k_2,)


@app.cell
def _(dct):
    for v in dct.values():
        print(v)
    return (v,)


@app.cell(hide_code=True)
def _(mo):
    mo.md(
        r"""
        ## Object-Oriented Programming (OOP)

        OOP is a programming paradigm that organizes software design around objects, which encapsulate data and behavior[^1][^2]. It emphasizes principles like inheritance, encapsulation, abstraction, and polymorphism to create scalable, reusable, and maintainable code[^3][^4].

        [^1]: https://www.techtarget.com/searchapparchitecture/definition/object-oriented-programming-OOP
        [^2]: https://en.wikipedia.org/wiki/Object-Oriented_Programming
        [^3]: https://emeritus.org/blog/coding-what-is-object-oriented-programming/
        [^4]: https://www.freecodecamp.org/news/java-object-oriented-programming-system-principles-oops-concepts-for-beginners/
        """
    )
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(
        r"""
        ### Classes and Objects

        A **class** is a blueprint or template that defines the attributes _(state)_ and methods _(behavior)_ for a group of objects sharing the same characteristics[^1][^2]. An **object** is an instance of a class, containing specific values for the attributes and enabling interaction with the defined methods[^1][^2].

        [^1]: https://www.techtarget.com/whatis/definition/class
        [^2]: https://www.baeldung.com/cs/class-object-differences
        """
    )
    return


@app.cell
def _():
    # Define a class
    class Car:
        def __init__(self, brand, model, year):
            self.brand = brand
            self.model = model
            self.year = year

        def print_details(self):
            print(f"Brand: {self.brand}, Model: {self.model}, Year: {self.year}")

    # Create an object
    car = Car("Toyota", "Corolla", 2015)

    # Use the object
    car.print_details()
    return Car, car


@app.cell(hide_code=True)
def _(mo):
    mo.md(
        """
        ### Inheritance

        Inheritance is a mechanism in object-oriented programming where a new class (subclass) inherits and builds upon the properties and behavior of an existing class (superclass), allowing for code reuse and hierarchical relationships.
        """
    )
    return


@app.cell
def _():
    # Parent class
    class Vehicle:
        def __init__(self, brand, model):
            self.brand = brand
            self.model = model

        def print_details(self):
            print(f"Brand: {self.brand}, Model: {self.model}")

    # Child class inheriting from Vehicle
    class NewCar(Vehicle):
        def __init__(self, brand, model, year):
            super().__init__(brand, model)
            self.year = year

        def print_full_details(self):
            self.print_details()
            print(f"Year: {self.year}")

    # Create an object of the child class
    new_car = NewCar("Toyota", "Corolla", 2015)

    # Use the object
    new_car.print_full_details()
    return NewCar, Vehicle, new_car


@app.cell(hide_code=True)
def _(mo):
    mo.md(
        """
        > ::solar:danger-triangle-line-duotone:: **Caution**
        > 
        > To better understand the implications of inheritance and the Liskov Substitution Principle, consider reading about the Circle-Ellipse problem at [this reference](https://en.wikipedia.org/wiki/Circle%E2%80%93ellipse_problem).
        """
    )
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(
        """
        ### Polymorphism

        Polymorphism is the ability for objects of different types to be treated as instances of a common superclass, enabling methods to perform differently based on the object's actual type[^1][^2][^3][^4].

        [^1]: https://dev.to/mzunairtariq/exploring-polymorphism-understanding-flexibility-in-object-oriented-programming-5g35
        [^2]: https://www.techtarget.com/whatis/definition/polymorphism
        [^3]: https://www.bmc.com/blogs/polymorphism-programming/
        [^4]: https://www.techtarget.com/searchapparchitecture/tip/Understanding-the-role-of-polymorphism-in-OOP
        """
    )
    return


@app.cell
def _():
    # Define a parent class
    class Shape:
        def area(self):
            pass

    from math import pi

    # Define child classes
    class Square(Shape):
        def __init__(self, side):
            self.side = side

        def area(self):
            return self.side ** 2

    class Circle(Shape):
        def __init__(self, radius):
            self.radius = radius

        def area(self):
            return pi * (self.radius ** 2)

    # Polymorphic function
    def calculate_area(shape):
        return shape.area()

    # Create objects
    square = Square(4)
    circle = Circle(5)

    # Use polymorphism
    print(f"Square Area: {calculate_area(square)}")
    print(f"Circle Area: {calculate_area(circle)}")
    return Circle, Shape, Square, calculate_area, circle, pi, square


@app.cell(hide_code=True)
def _(mo):
    mo.md(
        """
        ### Encapsulation

        Encapsulation is the practice of hiding internal implementation details of an object from the outside world while exposing only necessary information through controlled interfaces, ensuring data integrity and security.
        """
    )
    return


@app.cell
def _():
    class BankAccount:
        def __init__(self):
            self.__balance = 0

        def deposit(self, amount):
            if amount > 0:
                self.__balance += amount
                print(f"Deposited ${amount}. Current balance is ${self.__balance}")
            else:
                print("Invalid deposit amount.")

        def get_balance(self):
            return self.__balance

    # Create an object
    account = BankAccount()

    # Deposit money
    account.deposit(1000)

    # Check balance
    print(f"Current Balance: ${account.get_balance()}")
    return BankAccount, account


@app.cell(hide_code=True)
def _(mo):
    mo.md(
        """
        ### Abstraction

        Python's declarative aspects, such as list comprehensions or generator expressions, rely on abstraction to simplify complex operations and hide implementation details. This abstraction allows developers to focus on what needs to be achieved rather than how it is done, making the code more expressive and easier to maintain.
        """
    )
    return


@app.cell
def _():
    from abc import ABC, abstractmethod

    # Inherit from ABC (Abstract Base Class)
    class AbstractPaymentMethod(ABC): 
        @abstractmethod
        def make_payment(self, amount):
            pass

        def validate_payment(self, amount):
            if amount <= 0:
                print("Invalid payment amount. Must be greater than zero.")
                return False
            print("Payment validated.")
            return True

    # Concrete classes
    class CreditCard(AbstractPaymentMethod):
        def make_payment(self, amount):
            if self.validate_payment(amount):
                print(f"Paid ${amount} using Credit Card.")

    class PayPal(AbstractPaymentMethod):
        def make_payment(self, amount):
            if self.validate_payment(amount):
                print(f"Paid ${amount} using PayPal.")

    # Create objects
    credit_card = CreditCard()
    paypal = PayPal()

    # Use abstraction
    credit_card.make_payment(100)  # Valid payment
    credit_card.make_payment(-10)  # Invalid payment
    paypal.make_payment(50)        # Valid payment
    return (
        ABC,
        AbstractPaymentMethod,
        CreditCard,
        PayPal,
        abstractmethod,
        credit_card,
        paypal,
    )


@app.cell(hide_code=True)
def _(mo):
    mo.md("""### Managing Data Access""")
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""#### Traditional Getter and Setter Methods""")
    return


@app.cell
def _():
    class Person:
        def __init__(self, name):
            self._name = name  # Private attribute

        # Getter method
        def get_name(self):
            return self._name

        # Setter method
        def set_name(self, value):
            if not isinstance(value, str):
                raise TypeError("Name must be a string")
            self._name = value

    # Example usage
    person = Person("Said")
    print(person.get_name())  # Output: Said
    person.set_name("Ahmed")
    print(person.get_name())  # Output: Ahmed
    return Person, person


@app.cell(hide_code=True)
def _(mo):
    mo.md(
        r"""
        #### Using `@property` Decorator
        The `@property` decorator simplifies getter and setter methods by allowing attribute access like regular variables. Setters can include validation logic to ensure proper values are assigned.
        """
    )
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""##### Example \#1""")
    return


@app.cell
def _():
    class NewPerson:
        def __init__(self, name):
            self._name = name  # Private attribute

        @property
        def name(self):  # Getter method
            return self._name

        @name.setter
        def name(self, value):  # Setter method
            if not isinstance(value, str):
                raise TypeError("Name must be a string")
            self._name = value

    # Example usage
    new_person = NewPerson("Said")
    print(new_person.name)  # Output: Said (getter)
    new_person.name = "Ahmed"  # Setter usage
    print(new_person.name)  # Output: Ahmed

    try:
        new_person.name = 123  # Raises TypeError
    except TypeError as e:
        print(e)  # Output: Name must be a string
    return NewPerson, new_person


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""##### Example \#2""")
    return


@app.cell
def _():
    class Product:
        def __init__(self, price):
            self._price = price

        @property
        def price(self):  # Getter method
            return self._price

        @price.setter
        def price(self, value):  # Setter method with validation
            if value < 0:
                raise ValueError("Price cannot be negative")
            self._price = value

    # Example usage
    product = Product(100)
    print(product.price)  # Output: 100 (getter)
    product.price = 200   # Setter usage
    print(product.price)  # Output: 200

    try:
        product.price = -50  # Raises ValueError
    except ValueError as e:
        print(e)  # Output: Price cannot be negative
    return Product, product


@app.cell(hide_code=True)
def _(mo):
    mo.md(
        r"""
        #### Combining Property with Read-Only Attributes
        We can make attributes read-only by defining only a getter without a setter.
        """
    )
    return


@app.cell
def _():
    class NewCircle:
        def __init__(self, radius):
            self._radius = radius

        @property
        def radius(self):  # Read-only property
            return self._radius

    # Example usage
    new_circle = NewCircle(10)
    print(new_circle.radius)  # Output: 10

    try:
        new_circle.radius = 20  # Raises AttributeError because there's no setter defined.
    except AttributeError as e:
        print(e)  # Output: can't set attribute
    return NewCircle, new_circle


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""### Instance and Class Data""")
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(
        r"""
        #### Instance-Level Data (Instance Variables)

        - **Definition:** Instance variables are attributes that belong to each instance of a class. Each instance has its own copy of these variables. They are used when the value of a variable varies from object to object.
        - **Declaration:** Instance variables are declared inside methods, typically within the `__init__` method.
        - **Access:** They are accessed using the object reference (e.g., `self` within the class).
        """
    )
    return


@app.cell
def _():
    class Student:
        def __init__(self, name, age):
            self.name = name  # Instance variable
            self.age = age    # Instance variable

    # Creating instances
    student1 = Student("Roua", 20)
    student2 = Student("Rami", 22)

    print(student1.name)  # Output: Roua
    print(student2.name)  # Output: Rami
    return Student, student1, student2


@app.cell(hide_code=True)
def _(mo):
    mo.md(
        r"""
        #### Class-Level Data (Class Variables)

        - **Definition:** Class variables are shared attributes among all instances of a class. There is only one copy of these variables, which is shared by all instances. They are used when the value of a variable is consistent across all instances, or when you want to maintain a shared state.
        - **Declaration:** Class variables are declared outside any method, directly within the class definition.
        - **Access:** They can be accessed using either the class name or an instance of the class.
        """
    )
    return


@app.cell
def _():
    class School:
        ORG_NAME = "ISETBZ"  # Class variable

        def __init__(self, emp_id, name, salary):
            self.emp_id = emp_id   # Instance variable
            self.name = name       # Instance variable
            self.salary = salary   # Instance variable

    # Creating instances
    employee1 = School(1, "Ala", 1000)
    employee2 = School(2, "Eya", 1500)

    print(employee1.ORG_NAME)  # Output: ISETBZ
    print(employee2.ORG_NAME)  # Output: ISETBZ

    # Modifying the class variable affects all instances
    School.ORG_NAME = "ENIB"
    print(employee1.ORG_NAME)  # Output: ENIB
    print(employee2.ORG_NAME)  # Output: ENIB

    # Modifying the instance variable affects only the object itself
    employee1.ORG_NAME = "FSB"
    print(employee1.ORG_NAME)  # Output: FSB (creates a new instance variable)
    print(employee2.ORG_NAME)  # Output: ENIB

    School.ORG_NAME = "ISG"
    print(employee1.ORG_NAME)  # Output: FSB (class variable shadowed )
    print(employee2.ORG_NAME)  # Output: ISG
    return School, employee1, employee2


@app.cell(hide_code=True)
def _(mo):
    mo.md(
        r"""
        #### Key Differences
        ___
        | Feature            | Instance Variables      | Class Variables          |
        |--------------------|-------------------------|---------------------------|
        | **Ownership**      | Belong to each instance  | Shared by all instances   |
        | **Declaration**    | Inside methods (e.g., `__init__`) | Outside methods, in class definition |
        | **Access**         | Via instance reference (`self`) | Via class or instance reference |
        | **Usage**          | Unique per instance      | Shared across all instances |
        """
    )
    return


@app.cell
def _():
    import marimo as mo
    return (mo,)


if __name__ == "__main__":
    app.run()
