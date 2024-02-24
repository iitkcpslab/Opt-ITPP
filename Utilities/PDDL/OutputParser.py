import argparse
import json

def is_float(element):
    try:
        float(element)
        return float(element)
    except ValueError:
        return -1

def parse_location(loc):
    return (int(loc[1:3]), int(loc[3:5]))
    
def pddl_output_parser(file_location, planner):
    try:
        fp = open(file_location, 'r')
        if 'enhsp' in planner:
            flag = None
            results = []
            for l in fp:
                if flag:
                    results.append(l)
                if l.startswith('Problem Solved'):
                    flag = True
                if l.startswith('Problem Detected as Unsolvable'):
                    flag = False
                    
            if flag == True:
                plans = {}
                output = {}
                for line in results:
                    key, value = line.split(':')
                    if is_float(key) != -1:
                        value = value[2:-2]
                        action, agent, location, other = value.split(' ')
                        if agent not in plans:
                            plans[agent] = []
                            
                        location = parse_location(location)
                        if other[0] == 'l':
                            other = parse_location(other)
                        plans[agent].append([is_float(key), action, location, other])
                    else:
                        output[key] = value[:-1]
                output['Plans'] = plans
            if flag == False:
                output = {
                    'Status' : False,
                    'Reason' : 'Unsolvable'
                }
            if flag == None:
                output = {
                    'Status' : False,
                    'Reason' : 'Timeout'
                }
            return output
    
        elif 'ff' in planner:
            flag = None
            results = []
            output  = {}
            for l in fp:
                if flag:
                    results.append(l)
                if l.startswith('ff: found legal plan as follows'):
                    flag = True
                if l.startswith('ff: goal can be simplified to FALSE. No plan will solve it'):
                    flag = False
              
            if flag == True:
                plans = {}
                i = 0
                while i < len(results):
                    line = results[i]
                    if 'REACH-GOAL' in line:
                        i += 1
                        break
                    if i == 0:
                        line = line[4:]
                    while line[0] == ' ':
                        line = line[1:]
                    key, value = line.split(':')
                    value = value[1:-1]
                    action, agent, location, other = value.lower().split(' ')
                    if agent not in plans:
                        plans[agent] = []
                            
                    location = parse_location(location)
                    if other[0] == 'l':
                        other = parse_location(other)
                    plans[agent].append([is_float(key), action, location, other])
                    i += 1
                    
                output['Plans'] = plans
                
                key, value = results[i].split(':')
                output[key] = is_float(value)
                i += 2
                
                while len(results[i]) > 1:
                    line = results[i][:-1]
                    if line.startswith('time'):
                        line = line[11:]
                    while line[0] == ' ':
                        line = line[1:]
                    line = line.split(' ')
                    time, key = is_float(line[0]), ' '.join(line[2:]) + ' ' +line[1] 
                    output[key] = time
                    i += 1
                
                
            if flag == False:
                output = {
                    'Status' : False,
                    'Reason' : 'Unsolvable'
                }
            if flag == None:
                output = {
                    'Status' : False,
                    'Reason' : 'Timeout'
                }
            return output   
    except Exception as e:
        raise e
    finally:
        if fp:
            fp.close()
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("input")
    parser.add_argument("output")
    parser.add_argument("planner")

    args = parser.parse_args()

    input = args.input
    output = args.output
    planner = args.planner
   
    res = pddl_output_parser(input, planner)
    with open(output, 'w+') as fp:
        json.dump(res, fp)

