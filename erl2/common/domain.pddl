(define (domain cluedo_dom)

(:requirements :strips :typing :equality :fluents :disjunctive-preconditions)

(:types
    waypoint
    height
)

(:predicates
    (arm_initial_pos)
    (arm_pos_adjusted)
    (arm_at ?h - height)
    (base_at ?wp - waypoint)
    (center ?wp - waypoint)
    (unexplored ?wp - waypoint ?h - height)
    (correct_hyp)
)

(:functions
    (hint_number)
)

(:action start_game
    :parameters (?h - height)
    :precondition (and (arm_initial_pos))
    :effect (and
        (not (arm_initial_pos))
        (arm_pos_adjusted)
        (arm_at ?h)
    )
)


(:action goto_waypoint
    :parameters (?from ?to - waypoint)
    :precondition (and (base_at ?from) (arm_pos_adjusted))
    :effect (and
        (base_at ?to)
        (not (base_at ?from))
    )
)

(:action move_arm
    :parameters (?fromh ?toh - height)
    :precondition (and (arm_at ?fromh))
    :effect (and
        (arm_at ?toh)
        (not (arm_at ?fromh))
    )
)

(:action get_hint
    :parameters (?wp - waypoint ?h - height)
    :precondition (and (base_at ?wp) (arm_at ?h) (unexplored ?wp ?h))
    :effect (and
        (increase (hint_number) 1.0)
        (not (unexplored ?wp ?h))
    )
)


(:action check_hyp
    :parameters (?wp - waypoint)
    :precondition (and (>= (hint_number) 3.0) (center ?wp) (base_at ?wp) )
    :effect (and
        (correct_hyp)
        (assign (hint_number) 0.0)
    )
)



)
