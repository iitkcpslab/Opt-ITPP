To execute the problem using Classical Planner.

Pre-requisite:
    -> Install a classical planner supporting 
        (:requirements :strips :typing :fluents :adl)
        ex : https://gitlab.com/enricos83/ENHSP-Public
    
    -> domain.pddl is provided in this directory

    -> generatePDDL.py generates problem.pddl 
        from problem.json generated using ProblemGenerator.

    -> executePDDL.py executes using planner, domain.pddl and problem.pddl

    -> A sample JupyterNotebook contains helper script for above scripts.
        Above scripts supports command line executions as well.