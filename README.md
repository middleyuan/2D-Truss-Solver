# Truss-Calculator

This is a program that can compute member forces, reaction forces, and failures of the members of the truss defined in a `.txt` file.

## Methodology

* Utilize `method of joints` to form equilibrium equations of each node.
* Gather all the equilibrium equations from each node to form the whole system equation such that the member forces and reaction forces can be solved.
* Use the member force to determine if the member fail because of yielding or buckling.

## Truss definition

```
N1 Free 4 3 # x = 4, y = 3
N2 Loose 45 4 0 # angle(deg) = 45, x = 4, y = 0
N3 Fixed 0 0
----
N1-N2 I 0.1 0.2 0.01 210000000000 340000000 # B = 0.1, H = 0.2, Thickness = b = h = 0.01, young's modulus = 210000000000, yield strength = 340000000
N1-N3 T 0.1 0.2 0.01 210000000000 340000000
N2-N3 C 0.1 0.2 0.01 210000000000 340000000
----
N1 30000 0 # external force acts on N1; amplitide = 30000, angle (deg) = 0
```

* Member shape

    * The reference page [link](https://amesweb.info/section/second-moment-of-area-calculator.aspx) is used to define the parameter of the shape.  Note that we assume `b=h=thichness`.

## Setup and run

To run this program, do the steps in the following:

Clone this repository

    $ git clone https://git.rwth-aachen.de/dsme-projects/info-1/truss-calculator.git

If you want to use anaconda, you can create the environment from [truss-calculator.yaml](truss-calculator.yaml)

    $ conda env create -f truss-calculator.yml

Activate the enviornment

    $ conda activate "truss-calculator"

run one example file (e.g. truss2.txt)

    $ python main.py truss2.txt

sample output
```
member N1-N2 force = [-22500.] ; the member fails:  False
member N1-N3 force = [37500.] ; the member fails:  False
member N2-N3 force = [-22500.] ; the member fails:  False
reaction force of node N2 = 31819.80515339463
reaction force of node N3 in x direction = -7499.999999999988
reaction force of node N3 in y direction = -22500.0
```

## Unittest
run the `test_truss.py`, it will test from `truss1.txt` to `truss6.txt` cases.

    $ python test_truss.py




