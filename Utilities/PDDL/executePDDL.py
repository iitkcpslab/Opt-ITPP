import argparse
import subprocess
from typing import final
from generatePDDL import generate_pddl
import os


def pddl_execute(domain, problem, output, planner, offsetZ):
    
    generate_pddl('temporary_problem.pddl', problem, offsetZ)
    # print('PDDL Generated\nStarting Planner')
    # print(os.getcwd())
    # print(os.listdir(os.getcwd()))
    f = open(output, 'w+')
    f.write('Start')
    f.close()
    if 'enhsp' in planner:
        task = 'java -jar ' + str(planner) + ' -o ' +  domain + ' -timeout 3600 -f temporary_problem.pddl -planner opt-hmax  > ' + output
        try:
            planner = subprocess.Popen(task, shell=True)
            planner.communicate()
        except Exception as e:
            pass
        
    elif 'ff' in planner:
        task = planner+' -p ./ -o domain.pddl -f temporary_problem.pddl -s 0 >> ' + output
        try:
            planner = subprocess.Popen(task, shell=True)
            planner.communicate()
        except:
            pass
    # print('Planning Complete')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("domain_file")
    parser.add_argument("problem_file")
    parser.add_argument("output_file")
    parser.add_argument("planner")

    args = parser.parse_args()

    domain = args.domain_file
    problem = args.problem_file
    output = args.output_file
    planner = args.planner
   
    pddl_execute(domain, problem, output, planner)


