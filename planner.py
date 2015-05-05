# planner.py
from collections import namedtuple
from heapq import heappush, heappop
import json
with open('Crafting.json') as f:
    Crafting = json.load(f)

Items = Crafting['Items']
Recipes = Crafting['Recipes']
hasBench = False
hasFurnace = False
hasWoodenPickaxe = False
hasWoodenAxe = False
hasStonePickaxe = False
hasStoneAxe = False
hasIronPickaxe = False
hasIronAxe = False

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
for name, rule in Recipes.items():
    checker = make_checker(rule)
    effector = make_effector(rule)
    recipe = Recipe(name, checker, effector, rule['Time'])
    all_recipes.append(recipe)

def graph(state):
    for r in all_recipes:
        if r.check(state):
            yield (r.cost, r.effect(state), r.name)

'''def rawMaterials(currentState, wantedState):
    tot_req = wantedState
    currState = currentState
    requirements = tuple_subtract(currState, tot_req)
    # if you do not have a goal object, it should show up as a negative number
    for i in requirements:
        if i > 0:
            i = 0
        if i < 0:
            i = -i
    
            
    return None'''
        
def getActionsToTake(goals):
    goalActionsToTake = {}
    for i in goals:
        for key in Crafting['Recipes'].items():
            if key[1].get('Produces') and i in key[1].get('Produces'):
                goalActionsToTake[i] = key
    return goalActionsToTake
            
def heuristic(state, action):
    # ... 
    theGoal = Crafting['Goal'].keys()
    goalActions = getActionsToTake(theGoal)
    
    actionsToTake = goalActions.values()
    
    
    
    secondLevelGoal = theGoal
    
    
    
    for a, i in goalActions.values():
        if i.get('Requires'):
            for j in i.get('Requires').keys():
                secondLevelGoal.append(j)
        if i.get('Consumes'):
            for j in i.get('Consumes').keys():
                secondLevelGoal.append(j)
                
    
    
    secondLevelActions = getActionsToTake(secondLevelGoal).values()
    
    
    
    secondLevelOnlyActions = []
    
    for i in secondLevelActions:
        secondLevelOnlyActions.append(i[0])
        
    '''print '*********************'
    print secondLevelOnlyActions
    print '*********************'
    print " "'''
    
    
    stateDict = tuple_to_inventory(state)
    recipesReq = Recipes[action].get('Requires')
    recipesPro = Recipes[action].get('Produces')
    global hasBench
    global hasFurnace
    global hasWoodenPickaxe
    global hasWoodenAxe
    global hasStonePickaxe
    global hasStoneAxe
    global hasIronPickaxe
    global hasIronAxe
    
    if 'bench' in stateDict:
        if action is "craft bench":
            return 10000
        if not hasBench:
            hasBench = True
    else:
        if action is "craft bench":
            return -10000
            
    if 'furnace' in stateDict:
        if action is "craft furnace at bench":
            return 10000
        if action is "craft bench":
            return 10000
        if not hasFurnace:
            hasFurnace = True
    else:
        if action is "craft furnace at bench":
            return -10000
    
    if 'iron_pickaxe' in stateDict:
        if recipesReq and 'stone_pickaxe' in recipesReq:
            return 10000
        if recipesReq and 'wooden_pickaxe' in recipesReq:
            return 10000
        if action is "craft iron_pickaxe at bench":
            return 10000
        if action is "craft stone_pickaxe at bench":
            return 10000
        if action is "craft wooden_pickaxe at bench":
            return 10000
        if action is "craft bench":
            return 10000
        if not hasIronPickaxe:
            hasIronPickaxe = True
        
    else:
        if hasIronPickaxe:
            return 10000
            
    if 'stone_pickaxe' in stateDict:
        if recipesReq and 'wooden_pickaxe' in recipesReq:
            return 10000
        if action is "craft stone_pickaxe at bench":
            return 10000
        if action is "craft wooden_pickaxe at bench":
            return 10000
        if action is "craft bench":
            return 10000
        if not hasStonePickaxe:
            hasStonePickaxe = True
    else:
        if hasStonePickaxe:
            return 10000
    
    if 'iron_axe' in stateDict:
        if recipesReq and 'stone_axe' in recipesReq:
            return 10000
        if recipesReq and 'wooden_axe' in recipesReq:
            return 10000
        if action is "punch for wood":
            return 10000
        if action is "craft iron_axe at bench":
            return 10000
        if action is "craft stone_axe at bench":
            return 10000
        if action is "craft wooden_axe at bench":
            return 10000
        if action is "craft bench":
            return 10000
        if not hasIronAxe:
            hasIronAxe = True
        
    else:
        if hasIronAxe:
            return 10000
    
    if 'stone_axe' in stateDict:
        if recipesReq and 'wooden_axe' in recipesReq:
            return 10000
        if action is "punch for wood":
            return 10000
        if action is "craft stone_axe at bench":
            return 10000
        if action is "craft wooden_axe at bench":
            return 10000
        if action is "craft bench":
            return 10000
        if not hasStoneAxe:
            hasStoneAxe = True
        
    else:
        if hasStoneAxe:
            return 10000
    
    if 'wooden_axe' in stateDict:
        if action is "punch for wood":
            return 10000
        if action is "craft wooden_axe at bench":
            return 10000
        if action is "craft bench":
            return 10000
        if not hasWoodenAxe:
            hasWoodenAxe = True
    else:
        if hasWoodenAxe:
            return 10000
    
    maxItems = 8
    
    '''for i in Crafting['Goal'].values():
        if i > maxItems:
            maxItems = i'''
    
    for i in stateDict:
        if stateDict[i] > maxItems:
            if recipesPro:
                if i in recipesPro:
                    return 10000
                    
    if action in secondLevelOnlyActions:
        return -1000
        
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
                priority = alt+heuristic(n_state_t, n_action)
                if prev[(d_action, u, d_cost)] is not None:
                    prevac, prevst, prevco = prev[(d_action, u, d_cost)]
                    if prevac is not d_action:
                        dist[n_state_t] = priority
                        prev[(n_action, n_state_t, priority)] = (d_action, u, d_cost)
                        #handle if already in queue
                        heappush(queue, (dist[n_state_t], n_state_t, n_action))
                else:
                    dist[n_state_t] = priority
                    prev[(n_action, n_state_t, priority)] = (d_action, u, d_cost)
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
            newd = dist[s]
            plan.insert(0, (a, actual_state, newd))
            add = prev[add]
    return total_cost, plan

tot_cost, fullplan = search(graph, Crafting['Initial'], make_goal_checker(Crafting['Goal']), 2000, heuristic)
runningcost = 0
for i in fullplan:
    act, st, time = i
    runningcost += Recipes[act].get('Time')
    print str(act) + " " + str(st) + " " + str(runningcost)
    
print str(runningcost)