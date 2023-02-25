(define (problem cluedo_prob)
(:domain cluedo_dom)
(:objects 
    wp0 wp1 wp2 wp3 wp4 - waypoint
    h1 h2 - height
)

(:init
    (arm_initial_pos)
    (base_at wp0)
    (center wp0)
    (= (hint_percieved) 0.0)
    (unexplored wp1 h1)
    (unexplored wp1 h2)
    (unexplored wp2 h1)
    (unexplored wp2 h2)
    (unexplored wp3 h1)
    (unexplored wp3 h2)
    (unexplored wp4 h1)
    (unexplored wp4 h2)
    

)

(:goal (and
    (correct_hyp)
    )
)

)
