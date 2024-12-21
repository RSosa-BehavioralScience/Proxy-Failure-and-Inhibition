#-----------------------------------------------------------------------------
# GOAL PURSUIT VIA TRACES
#-----------------------------------------------------------------------------

#Setup and Package Loading ----
if (!require('ggplot2')) install.packages('ggplot2'); library('ggplot2')
if (!require('dplyr')) install.packages('dplyr'); library('dplyr')
if (!require('tibble')) install.packages('tibble'); library('tibble')
if (!require('ggforce')) install.packages('ggforce'); library('ggforce')

#Set language to English for consistent output
Sys.setenv(LANG = "en")

#Helper Functions ----

#Bound Gain Values Between 0 and 1
#
#Used to control angle drift in movement.
gain_function <- function(x) {
  rate <- -log10(0.2)  
  1 - exp(-rate * x)
}

#Check Path-Goal Intersection
#
#Determines if a path segment intersects with the circular goal area
#...using geometric calculations to determine the end of a goal-pursuit episode
#prev_x and prev_y are previous position coordinates
#current_x and current_y are current position coordinates
#goal_x and goal_y are goal position coordinates
#goal_radius represents the radius of the goal area
#Returns logical value indicating whether path intersects goal
path_cross_goal <- function(prev_x, prev_y, current_x, current_y, goal_x, goal_y, goal_radius) {
  dx <- current_x - prev_x
  dy <- current_y - prev_y
  fx <- prev_x - goal_x
  fy <- prev_y - goal_y
  
  a <- dx * dx + dy * dy
  if (a == 0) return(FALSE)
  
  b <- 2 * (fx * dx + fy * dy)
  c <- fx * fx + fy * fy - goal_radius * goal_radius
  
  discriminant <- b * b - 4 * a * c
  if (discriminant < 0) {
    return(FALSE)
  } else {
    discriminant <- sqrt(discriminant)
    t1 <- (-b - discriminant) / (2 * a)
    t2 <- (-b + discriminant) / (2 * a)
    if ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1)) {
      return(TRUE)
    }
  }
  return(FALSE)
}

#Create Blob-like Shapes
#
#Generates irregular circular shapes by adding random variation to radius.
#Used to create environmental cues.
#x and y are center coordinates of the blob
#size represents the base size of the blob
#points represents the number of points to use in blob construction
#Returns a data frame with blob irregular polygon coordinates
create_blob <- function(x, y, size, points = 80) {
  angle <- seq(0, 2 * pi, length.out = points)
  radius <- size * (1 + 0.15 * runif(points))
  data.frame(
    x = x + radius * cos(angle),
    y = y + radius * sin(angle),
    center_x = x,
    center_y = y
  )
}

#Point in Polygon Test
# 
#Implements ray-casting algorithm to determine if a point lies within a 
#...polygon to determine if the agent is sensing a cue
#x and y are the agent's coordinates
#polygon is the data frame containing polygon parameters
#Returns a logical value indicating whether point is inside a certain polygon
point_in_polygon <- function(x, y, polygon) {
  n <- nrow(polygon)
  inside <- FALSE
  j <- n
  for (i in 1:n) {
    xi <- polygon$x[i]
    yi <- polygon$y[i]
    xj <- polygon$x[j]
    yj <- polygon$y[j]
    intersect <- ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)
    if (intersect) inside <- !inside
    j <- i
  }
  return(inside)
}

#Initial Parameters and Environment Setup ----

initial_step_length <- 1 #Determines the default length of agent's displacements
goal_x <- 9.5            #Goal center location X
goal_y <- 9.5            #Goal center location Y
goal_radius <- sqrt(0.75 / pi)  #Goal area radius
boundary_min <- 0        #Environment boundaries
boundary_max <- 10
tail_length <- 2         #Visual trail length

#Blob (environmental cue) setup
num_blobs <- 30
blob_sizes <- seq(from = 1, to = 12, length.out = 30)

#Position blobs
blob_x <- rep(goal_x, num_blobs) #Cues around the goal
blob_y <- rep(goal_y, num_blobs)

#Main Simulation Loop ----
steps_count_per_iteration <- vector()  #Store steps per episode
loops <- 400                           #Number of episodes

#Progress bar setup
pb <- txtProgressBar(min = 0,
                     max = loops,
                     style = 3,
                     width = 100,
                     char = "=")

set.seed(1401)  #Set random seed for reproducibility

#Iterate through specified number of episodes
for (i in 1:loops) {
  setTxtProgressBar(pb, i)  #Update progress bar
  
  #Create different set of random environmental cues (blobs) each episode
  blobs <- lapply(1:num_blobs, function(i) {
    create_blob(blob_x[i], blob_y[i], blob_sizes[i])
  })
  
  #Initialize episode starting conditions
  current_x <- boundary_min
  current_y <- boundary_min
  prev_cue <- 0              #Previous cue intensity
  prev_nf <- 0               #Previous negative feature count
  angle <- 0                 #Initial movement angle
  
  #Initialize data storage for current episode
  steps <- tibble(
    step = integer(),          #Step number
    x = numeric(),             #X position
    y = numeric(),             #Y position
    length = numeric(),        #Intended step length
    abs_angle = numeric(),     #Absolute angle (intended)
    rel_angle = numeric(),     #Relative angle change (intended)
    collision = logical(),     #Boundary collision flag
    true_length = numeric(),   #Actual step length after bouncing (if any)
    true_angle = numeric(),    #Actual movement angle after bouncing (if any)
    true_angle_r = numeric(),  #Relative true angle
    trace = numeric(),     #Proxy cue intensity (number of goal-related blobs)
    gain_trace = numeric(),
    aff = character(),         #Affective valence (approach/avoid)
    gain_trans = numeric()     #Adaptive gain transformed into proportion of angle constraint
  )
  
  #Initialize position tracking
  previous_x <- current_x
  previous_y <- current_y
  
  #Displacement Loop: Continue until goal is reached
  while (!path_cross_goal(previous_x, previous_y, current_x, current_y, goal_x, goal_y, goal_radius)) {
    #Update position tracking
    previous_x <- current_x
    previous_y <- current_y
    
    #Count proxy cues at current position
    inside_blobs <- sapply(1:num_blobs, function(i) {
      point_in_polygon(current_x, current_y, blobs[[i]])
    })
    inside_blob_count <- sum(inside_blobs)
    
    #Store cue encounters and calculate changes in environmental information
    gain <- (inside_blob_count - prev_cue)
    
    #Calculate combined gain and determine movement parameters
    gain_comp <- gain #* V #+ gain_nf * Vnf    #Combined gain from all features
    gain_f <- gain_function(abs(gain_comp))  #Transform gain for angle constraint
    
    #Determine affective state based on gain
    aff <- if (gain_comp < 0) pi else 0      #pi = avoid, 0 = approach
    
    #Update movement angle based on gain and affective state
    angle <- angle + runif(1, -pi*(1-gain_f)+aff, pi*(1-gain_f)+aff)
    
    #Calculate step length based on information gain
    length <- initial_step_length + log10(initial_step_length+(gain_f/0.1))
    
    #Calculate next position
    next_x <- current_x + length * cos(angle)
    next_y <- current_y + length * sin(angle)
    
    #Handle boundary collisions
    collision <- FALSE
    if (next_x < boundary_min) {
      next_x <- boundary_min + (boundary_min - next_x)
      collision <- TRUE
    } else if (next_x > boundary_max) {
      next_x <- boundary_max - (next_x - boundary_max)
      collision <- TRUE
    }
    
    if (next_y < boundary_min) {
      next_y <- boundary_min + (boundary_min - next_y)
      collision <- TRUE
    } else if (next_y > boundary_max) {
      next_y <- boundary_max - (next_y - boundary_max)
      collision <- TRUE
    }
    
    #Update current position
    current_x <- next_x
    current_y <- next_y
    
    #Calculate bounded angle for display (0 to 2π)
    bound_angle <- if (angle < 0) {
      2*pi + sign(angle)*acos(cos(angle))
    } else {
      acos(cos(angle))
    }
    
    #Calculate true displacement length
    true_length <- sqrt((current_x - previous_x)^2 + (current_y - previous_y)^2)
    
    #Calculate true movement angle
    if (current_x != previous_x) {
      delta_y <- current_y - previous_y
      delta_x <- current_x - previous_x
      angle <- atan2(delta_y, delta_x)
    } else {
      angle <- if (current_y > previous_y) pi/2 else 3*pi/2
    }
    
    true_angle_ <- if (angle < 0) {
      2*pi + sign(angle)*acos(cos(angle))
    } else {
      acos(cos(angle))
    }
    
    #Record current step data
    new_step <- tibble(
      step = nrow(steps) + 1,
      x = current_x,
      y = current_y,
      length = length,
      abs_angle = bound_angle * (180 / pi),
      rel_angle = ifelse(nrow(steps) > 0, 
                         bound_angle * (180 / pi) - steps$abs_angle[nrow(steps)], 
                         NA),
      collision = collision,
      true_length = true_length,
      true_angle = true_angle_ * (180 / pi),
      true_angle_r = ifelse(nrow(steps) > 0, 
                            (true_angle_ * (180 / pi)) - steps$true_angle[nrow(steps)], 
                            NA),
      trace = inside_blob_count,
      gain_trace = gain,
      aff = case_when(
        aff == 0 ~ "app",
        aff == pi ~ "av",
        TRUE ~ as.character(aff)
      ),
      #Adaptive gain transformed into proportion of angle constraint
      gain_trans = gain_f 
    )
    
    #Update step record
    steps <- bind_rows(steps, new_step)
    
    #Update tracking variables for next iteration
    prev_cue <- inside_blob_count
  }

  #Store episode performance metric
  steps_count_per_iteration[i] <- nrow(steps)
}

#Adjust the outcome data frame to enhance readability and facilitate inspection
steps$trace <- c(steps$trace[-1], 30)
steps$aff <- c(steps$aff[-1], NA)
steps$gain_trans <- c(steps$gain_trans[-1], NA)
steps$gain_trans <- format(steps$gain_trans, scientific = FALSE)

#Basic inspection of the output

#Visualize the temporal progression of steps per episode, which should be
#... fairly stationary
plot(steps_count_per_iteration, type = "l",
     main = "Progression Over Episodes",
     xlab = "Episode Number",
     ylab = "Steps to Goal",
     col = "steelblue")

#Calculate the typical performance metric using median
typical_performance <- median(steps_count_per_iteration)
print(paste("Typical steps per episode:", typical_performance))

#Find an episode that represents typical performance
#We look for the episode with steps count closest to but lower than
#...the median (507 steps) to illustrate a representative pattern
representative_episode <- which(steps_count_per_iteration == 
                                  min(steps_count_per_iteration[steps_count_per_iteration > typical_performance]))
print(paste("Representative episode number:", representative_episode))

#Plot the agent's path during the last episode
plot(c(0, steps$x), c(0, steps$y), type = "l",
     main = "Agent Trajectory in the Last Episode",
     xlab = "X Position",
     ylab = "Y Position",
     col = "darkgreen")

#Create a density plot of steps per episode
plot(density(steps_count_per_iteration, adjust = 0.5),
     main = "Distribution of Steps to Goal",
     xlab = "Number of Steps",
     ylab = "Density",
     col = "darkred")

#Store the vector containing the distribution of steps per episode
steps_trace <- steps_count_per_iteration

#-----------------------------------------------------------------------------
# VISUALIZATION
#-----------------------------------------------------------------------------

#To generate the trajectory plot shown in Figure 1:
#Set loops <- 396 to capture the representative episode
#Execute until the simulation loop (stopping before the analysis section)
#Run the command below to store the agent's path

rep_episode_trace <- steps

#Prepare data for plotting
blob_plot_data <- do.call(rbind, lapply(1:num_blobs, function(i) {
  blob <- blobs[[i]]
  blob$id <- num_blobs - i + 1  # Reverse the id numbering
  blob
}))

#Clip blobs to the visible region [0, 10]
blob_plot_data <- blob_plot_data %>%
  arrange(id) %>%  
  mutate(
    x = pmax(pmin(x, 10), 0),  #Constrain x within [0, 10]
    y = pmax(pmin(y, 10), 0)   #Constrain y within [0, 10]
  )

#Create first row with appropriate values for all columns
new_row <- tibble(
  step = 0,           
  x = 0,              #Starting x position
  y = 0,              #Starting y position
  length = NA,         
  abs_angle = NA,      
  rel_angle = NA,     
  collision = NA,  
  true_length = NA,   
  true_angle = NA,     
  true_angle_r = NA,  
  trace = NA,     
  gain_trace = NA,
  aff = NA,      
  gain_trans = NA      
)

#Combine the new row with existing data
rep_episode_trace <- bind_rows(new_row, rep_episode_trace)

#Plot blobs and agent position with goal trace
agent_path_trace <- ggplot(blob_plot_data, aes(x = x, y = y, group = id)) +
  geom_polygon(aes(fill = "Goal trace"), alpha = 0.05, color = "gray") +  
  geom_circle(aes(x0 = goal_x, y0 = goal_y, r = goal_radius), fill = "red4", alpha = 1) +
  #Add agent displacement path
  geom_path(data = rep_episode_trace, aes(x = x, y = y, group = 1), 
            color = "darkgreen", alpha = 1, size = 0.75) +
  scale_x_continuous(limits = c(0, 10), breaks = seq(0, 10, 1), expand = c(0, 0)) +
  scale_y_continuous(limits = c(0, 10), breaks = seq(0, 10, 1), expand = c(0, 0)) +
  theme_minimal() +
  theme(
    axis.text.x = element_text(size = 12),
    axis.text.y = element_text(size = 12),
    axis.title.x = element_text(size = 14),
    axis.title.y = element_text(size = 14),
    panel.border = element_rect(color = "black", fill = NA, size = 0.5),
    legend.text = element_text(size = 12),
    legend.title = element_blank()
  ) +
  labs(
    x = "X Position", 
    y = "Y Position"
  ) +
  scale_fill_manual(
    values = c("Goal trace" = "red"),
    guide = guide_legend(override.aes = list(alpha = 1))
  ) +
  coord_fixed()

print(agent_path_trace)
#Recommended dimensions: 800 × 650 pixels

#Comparison of numer of steps distribution in unguided and trace conditions

#Raw data
ggplot() +
  geom_density(aes(x = steps_unguided, fill = "Unguided"), alpha = 0.5) +
  geom_density(aes(x = steps_trace, fill = "With Trace"), alpha = 0.5) +
  labs(x = "Number of Steps", y = "Density") +
  theme_minimal()

#Log scale
unguided_trace_dist <- ggplot() +
  geom_density(aes(x = steps_unguided, fill = "Unguided"), alpha = 0.5) +
  geom_density(aes(x = steps_trace, fill = "With Trace"), alpha = 0.3) +
  scale_x_log10(
    limits = c(5, 4000), 
    breaks = c(0.1, 1:9, 10*(1:9), 100*(1:10), 2000, 3000, 4000)
  ) +
  labs(
    x = "Number of Steps to Attain Goal (log scale)", 
    y = "Density", 
    fill = "Condition"
  ) +
  theme_minimal(base_size = 20) + # Globally doubles text size
  theme(
    panel.grid = element_blank(),
    axis.text.y = element_blank(),
    panel.border = element_rect(color = "black", fill = NA, size = 0.5),
    legend.position = c(0.8, 0.8),
    legend.background = element_rect(fill = "white", color = "black", size = 0.2),
    legend.margin = margin(6, 6, 6, 6),
    axis.title = element_text(size = 15),      
    axis.text = element_text(size = 7),        
    legend.title = element_text(size = 12),    
    legend.text = element_text(size = 10)      
  ) +
  scale_fill_manual(values = c("Unguided" = "gray", "With Trace" = "red"))

print(unguided_trace_dist)
#Recommended dimensions: 1500 × 650 pixels

#Continue by running script number 3
