#-----------------------------------------------------------------------------
# GOAL ATTAINMENT DEVOID OF ANY GOAL TRACE
#-----------------------------------------------------------------------------

# Description:
#   This script simulates the random movement of an agent within a two-dimensional
#   environment. Starting from a specified initial position, the agent reaches
#   a predefined goal solely via random displacement, without any external
#   cues or directional guidance.

###############################################################################

#Setup and Package Loading ----
if (!require('ggplot2')) install.packages('ggplot2'); library('ggplot2')
if (!require('dplyr')) install.packages('dplyr'); library('dplyr')
if (!require('tibble')) install.packages('tibble'); library('tibble')
if (!require('ggforce')) install.packages('ggforce'); library('ggforce')

#Set language to English for consistent output
Sys.setenv(LANG = "en")

#Helper Functions ----

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

#Initial Parameters and Environment Setup ----

initial_step_length <- 1 #Determines the default length of agent's displacements
goal_x <- 9.5            #Goal center location X
goal_y <- 9.5            #Goal center location Y
goal_radius <- sqrt(0.75 / pi)  #Goal area radius
boundary_min <- 0        #Environment boundaries
boundary_max <- 10
tail_length <- 2         #Visual trail length

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
    true_angle_r = numeric()   #Relative true angle
  )
  
  #Initialize position tracking
  previous_x <- current_x
  previous_y <- current_y
  
  #Set step counter to zero
  k <- 0
  
  #Displacement Loop: Continue until goal is reached
  while (!path_cross_goal(previous_x, previous_y, current_x, current_y, goal_x, goal_y, goal_radius)) {
    #Add a unity to the step counter
    k <- k + 1
    
    #Update position tracking
    previous_x <- current_x
    previous_y <- current_y
    
    #Update movement angle by selecting a random displacement between 0° and 360°
    angle <- angle + runif(1, -pi, pi)
    
    #Set step length as a constant
    length <- initial_step_length 
    
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
    
    if (i == loops){
      #Record step data in the last loop
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
                              NA)
      )
      
      #Update step record
      steps <- bind_rows(steps, new_step)
    }
    
  }

  #Store episode performance metric
  steps_count_per_iteration[i] <- k
}

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

#Find an episode that represents typical performance ########################################################
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
steps_unguided <- steps_count_per_iteration

#-----------------------------------------------------------------------------
# VISUALIZATION
#-----------------------------------------------------------------------------

#To generate the trajectory plot shown in Figure 1:
#Set loops <- 85 to capture the representative episode
#Execute until the simulation loop (stopping before the analysis section)
#Run the command below to store the agent's path

rep_episode_unguided <- steps

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
  )

#Combine the new row with existing data 
rep_episode_unguided <- bind_rows(new_row, rep_episode_unguided)

#Plot agent displacement path to goal
agent_path_unguided <- ggplot() +
  geom_circle(aes(x0 = goal_x, y0 = goal_y, r = goal_radius), fill = "red4", alpha = 1) +
  geom_path(data = rep_episode_unguided, aes(x = x, y = y, group = 1), 
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
  coord_fixed()

print(agent_path_unguided)
#Recommended dimensions: 700 × 650 pixels

#Continue by running script number 2
