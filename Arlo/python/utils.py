def should_stop(front_dist, left_dist, right_dist):        
    return (
        (front_dist != -1 and front_dist < 500) or 
        (left_dist != -1 and left_dist < 200) or 
        (right_dist != -1 and right_dist < 200)
    )