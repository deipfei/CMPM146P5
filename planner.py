# planner.py
from collections import namedtuple
from heapq import heappush, heappop
import json
with open('Crafting.json') as f:
    Crafting = json.load(f)

Items = Crafting['Items']

def inventory_to_tuple(d):
    return tuple(d.get(name,0) for i,name in enumerate(Items))
    
def tuple_to_inventory(t):
    new_state = {}
    for i in range (0, len(t)):
        if t[i] != 0:
          new_state[Items[i]] = t[i]
    return new_state

def inventory_to_frozenset(d):
    return frozenset(d.items())

def tuple_subtract(t1, t2):
    for i in range (0, len(t1)):
        yield t1[i] - t2[i]

def tuple_add(t1, t2):
    for i in range (0, len(t1)):
        yield t1[i] + t2[i]

def make_initial_state(inventory):
    state = inventory
    return state

initial_state = make_initial_state(Crafting['Initial'])

def make_goal_checker(goal):
    goalstate = inventory_to_tuple(goal)
    def is_goal(state):
        curr = inventory_to_tuple(state)
        legal = True
        for i in range (0, len(curr)):
          if curr[i] < goalstate[i]:
            legal = False
            break
        return legal
    return is_goal

is_goal = make_goal_checker(Crafting['Goal'])

def make_checker(rule):
    con = rule.get('Consumes')
    req = rule.get('Requires')
    
    if con:
        consumes = inventory_to_tuple(con)
    if req:
        requires = req.keys()[0]
    def check(state):
        curr = inventory_to_tuple(state)
        legal = True
        if con:
            for i in range (0, len(curr)):
                if curr[i] < consumes[i]:
                    legal = False
                    break
        if req:
            if requires not in state:
                legal = False
        return legal
    return check

def make_effector(rule):
    con = rule.get('Consumes')
    pro = rule.get('Produces')
    if con:
        consumes = inventory_to_tuple(rule.get('Consumes'))
    if pro:
        produces = inventory_to_tuple(rule.get('Produces'))
    def effect(state):
        tuplestate = inventory_to_tuple(state)
        if con:
           tuplestate = tuple(tuple_subtract(tuplestate, consumes))
        if pro:
           tuplestate = tuple(tuple_add(tuplestate, produces))
        next_state = tuple_to_inventory(tuplestate)
        return next_state
    return effect        
    
Recipe = namedtuple('Recipe',['name','check','effect','cost'])
all_recipes = []
#print Crafting['Recipes']
for name, rule in Crafting['Recipes'].items():
    checker = make_checker(rule)
    effector = make_effector(rule)
    recipe = Recipe(name, checker, effector, rule['Time'])
    all_recipes.append(recipe)

def graph(state):
    for r in all_recipes:
        if r.check(state):
            yield (r.cost, r.effect(state), r.name)

def rawMaterials(item, start):
    tot_req = start
    reqs = Crafting['Recipes'][item].get('Requires')
    cons = Crafting['Recipes'][item].get('Consumes')
    #for i in mats:
    #    while reqs:
            
    return None
        
    
            
def heuristic(state, action):
    # ... 
    theGoal = Crafting['Goal']
    stateDict = tuple_to_inventory(state)
    recipesReq = Crafting['Recipes'][action].get('Requires')
    recipesPro = Crafting['Recipes'][action].get('Produces')
    if 'iron_pickaxe' in stateDict:
        if recipesReq and 'stone_pickaxe' in recipesReq:
            return 10000
        if recipesReq and 'wooden_pickaxe' in recipesReq:
            return 10000
    if 'stone_pickaxe' in stateDict:
        if recipesReq and 'wooden_pickaxe' in recipesReq:
            return 10000
    if 'iron_axe' in stateDict:
        if recipesReq and 'stone_axe' in recipesReq:
            return 10000
        if recipesReq and 'wooden_axe' in recipesReq:
            return 10000
        if action is "punch for wood":
            return 10000
    if 'stone_axe' in stateDict:
        if recipesReq and 'wooden_axe' in recipesReq:
            return 10000
        if action is "punch for wood":
            return 10000
    if 'bench' in stateDict:
        if action is "craft bench":
            return 10000
    if 'furnace' in stateDict:
        if action is "craft furnace":
            return 10000
    for i in stateDict:
        if stateDict[i] > 8:
            if recipesPro:
                if i in recipesPro:
                    return 10000
    return 0 # or something more accurate
    
def search(graph, initial, is_goal, limit, heuristic):
    queue = []
    dist = {}
    initial_t = inventory_to_tuple(initial)
    dist[initial_t] = 0
    prev = {}
    prev[("start", initial_t, 0)] = None
    heappush(queue, (dist[initial_t], initial_t, "start"))
    limitchecker = 0
    goalfound = False

    u = initial_t
    d_action = None
    d_cost = 0
    while queue:
        d_cost, u, d_action = heappop(queue)
        print d_action
        if(d_cost > limit):
            break
        u_state = tuple_to_inventory(u)
        if(is_goal(u_state)):
            goalfound = True
            break
        neighbors = graph(u_state)
        for next_node in neighbors:
            n_cost, n_state, n_action = next_node
            n_state_t = inventory_to_tuple(n_state)
            alt = d_cost + n_cost #changed to reflect total distance
            if n_state_t not in dist or alt < dist[n_state_t]:
                dist[n_state_t] = alt + heuristic(n_state_t, n_action)
                prev[(n_action, n_state_t, alt)] = (d_action, u, d_cost)
                #handle if already in queue
                heappush(queue, (dist[n_state_t], n_state_t, n_action))

    plan = []
    total_cost = 0
    if goalfound:
        total_cost = d_cost
        add = prev[(d_action, u, d_cost)]
        plan.insert(0, (d_action, u, d_cost))
        while add != ("start", initial_t, 0):
            a, s, t = add
            actual_state = tuple_to_inventory(s)
            plan.insert(0, (a, actual_state, t))
            add = prev[add]
    return total_cost, plan

tot_cost, fullplan = search(graph, Crafting['Initial'], make_goal_checker(Crafting['Goal']), 80, heuristic)
for i in fullplan:
    act, st, time = i
    print str(act) + " " + str(st) + " " + str(time)