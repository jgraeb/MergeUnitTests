# Apurva Badithela (8/4/21)
# Method to test winning sets from Ioannis' correspondence

import omega.symbolic.temporal as trl
from omega.games import gr1
from omega.games import enumeration as enum
import pdb

aut = trl.Automaton()
aut.declare_variables(x=(1, 3), y=(-3, 3))
aut.varlist.update(env=['x'], sys=['y'])
aut.init['env'] = 'x = 1'
aut.init['sys'] = 'y = 2'
aut.action['env'] = r'''
/\ x \in 1..2
/\ x' \in 1..2
'''
aut.action['sys'] = r'''
/\ y \in -3..3
/\ y' = x - 3
'''
aut.win['<>[]'] = aut.bdds_from('x = 2')
aut.win['[]<>'] = aut.bdds_from('y != -1')
aut.qinit = r'\E \A'
aut.moore = True
aut.plus_one = True
u = aut.add_expr(r'''(x \in 1..2)/\(y = 0)''')

z, yij, xijk = gr1.solve_streett_game(aut)
pdb.set_trace()
expr = aut.to_expr(z.to_bdd)
print("Winning set")
print(expr)


gr1.make_streett_transducer(z, yij, xijk, aut)
g = enum.action_to_steps(aut, env='env', sys='impl', qinit=aut.qinit)

