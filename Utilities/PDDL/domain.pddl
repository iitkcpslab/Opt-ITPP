(define (domain warehouse)
	(:requirements :strips :typing :fluents :adl)

	(:types
		agent task location - object
		robot - agent
		timer - fluent
	)

	(:predicates
		(adjecent ?l1 - location ?l2 - location)
		(robot-at ?r - robot ?l - location)
		(task-at ?t - task ?l - location)
		(carrying ?r - robot ?t - task)
		(is-task-goal-location ?t - task ?l - location)
		(is-intermediate-drop-location ?l - location)
		(blocked ?l - location)
	)

	(:functions
		(maxstep) - number
		(robot-capacity ?r - robot) - number
		(task-weight ?t - task) - number
		(task-deadlines ?t - task) - number
		(robot-time ?r - robot) - number
		(robot-action ?r - robot) - number 
	)

	(:action move
		:parameters (?agent - robot ?l1 - location ?l2 - location)
		:precondition (and
			(robot-at ?agent ?l1)
			(adjecent ?l1 ?l2)
			(not (blocked ?l2))
		)
		:effect (and
			(not (robot-at ?agent ?l1))
			(robot-at ?agent ?l2)
			(not (blocked ?l1))
			(blocked ?l2)
			(increase (total-time) 1)
			(increase (robot-time ?agent) 1)
		)
	)

	(:action pickup
		:parameters (?agent - robot ?l1 - location  ?t - task)
		:precondition (and
			(robot-at ?agent ?l1)
			(task-at ?t ?l1)
			(>= (robot-capacity ?agent) (task-weight ?t))
			(< (robot-action ?agent) (maxstep))
		)
		:effect (and
			(carrying ?agent ?t)
			(increase (total-time) 1)
			(increase (robot-time ?agent) 1)
			(decrease (robot-capacity ?agent) (task-weight ?t))
			(increase (robot-action ?agent) 1)
		)
	)

	(:action drop-at-goal
		:parameters (?agent - robot ?l1 - location ?t - task)
		:precondition (and
			(carrying ?agent ?t)
			(robot-at ?agent ?l1)
			(is-task-goal-location ?t ?l1)
			(< (robot-time ?agent) (task-deadlines ?t))
			(< (robot-action ?agent) (maxstep))
		)
		:effect (and
			(not (carrying ?agent ?t))
			(task-at ?t ?l1)
			(increase (total-time) 1)
			(increase (robot-time ?agent) 1)
			(increase (robot-capacity ?agent) (task-weight ?t))
			(increase (robot-action ?agent) 1)
		)
	)

	(:action drop-at-intermediate-location
		:parameters (?agent - robot ?l1 - location ?t - task)
		:precondition (and
			(carrying ?agent ?t)
			(robot-at ?agent ?l1)
			(is-intermediate-drop-location ?l1)
			(< (robot-action ?agent) (maxstep))
		)
		:effect (and
			(not (carrying ?agent ?t))
			(task-at ?t ?l1)
			(increase (total-time) 1)
			(increase (robot-time ?agent) 1)
			(increase (robot-capacity ?agent) (task-weight ?t))
			(increase (robot-action ?agent) 1)
		)
	)
)