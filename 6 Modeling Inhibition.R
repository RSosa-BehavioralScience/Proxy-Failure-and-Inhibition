#-----------------------------------------------------------------------------
# OVERCOMMING PROXY FAILURE THROUGH BEHAVIORAL INHIBITION
#-----------------------------------------------------------------------------

# Description:
#   This script employs a setup nearly identical to the previous one, with one 
#   crucial enhancement: the agent is now endowed with the ability to discern 
#   deceiving cues, as these are observed to be negatively correlated with goal 
#   attainment. Through this mechanism, the negative features of deceiving cues 
#   acquire properties that counteract goal pursuit, thereby enabling the agent 
#   to efficiently attain the goal by selectively following cues that lack these 
#   features.

###############################################################################

#Setup and Package Loading ----
if (!require('ggplot2')) install.packages('ggplot2'); library('ggplot2')
if (!require('dplyr')) install.packages('dplyr'); library('dplyr')
if (!require('tibble')) install.packages('tibble'); library('tibble')
if (!require('ggforce')) install.packages('ggforce'); library('ggforce')
if (!require('patchwork')) install.packages('patchwork'); library('patchwork')

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

#Main Simulation Loop ----
steps_count_per_iteration <- vector()  #Store steps per episode
loops <- 995                          #Number of episodes

#Learning parameters for goal proxy (recall that this cue appears both around the goal and at 
#...misleading sites)
PE <- 0                  #Prediction Error
V <- 0                   #Associative value
f_inc <- 0               #Feedback increment
lr_cue <- 0.03           #Learning rate for proxy

#Negative feature parameters
PEnf <- 0               #Prediction Error for negative feature
Vnf <- 0                #Associative value for negative feature
f_inc_nf <- 0           #Feedback increment for negative feature
lr_nf <- 0.03           #Learning rate for negative feature

#Associative value evolution tracking
V_evol <- c()          #Track associative values for goal associated cue
Vnf_evol <- c()        #Track negative feature associative values

#Other parameters of the agent and simulated environment
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
blob_sizes <- c(blob_sizes, rep(seq(from = 0.5, to = 6, length.out = 30), 2))

#Position blobs
blob_x <- rep(goal_x, num_blobs) #Cues around the goal
blob_y <- rep(goal_y, num_blobs)
blob_x <- c(blob_x, rep(0.5, 30), rep(9.5, 30))  #Misleading cue locations
blob_y <- c(blob_y, rep(9.5, 30), rep(0.5, 30)) 
num_blobs <- length(blob_sizes)  #Update total blob count

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
  
  #Identify negative feature blobs (indices 31 to end)
  nf <- blobs[31:num_blobs]
  
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
    proxy_cue = numeric(),     #Proxy cue intensity (number of goal-related blobs)
    gain_cue = numeric(),      #Change in cue intensity from step i-1
    neg_feat = numeric(),      #Negative feature count
    gain_feat = numeric(),     #Change in negative feature intensity from step i-1
    aff = character(),         #Affective valence (approach/avoid)
    gain_tot = numeric()       #Total gain (combining different sources of information)
  )
  
  #Calculate prediction errors and update associative values via a
  #...linear-operator rule assuming that elements gain or 
  #...lose associative strength *independently* as a function of the 
  #...discrepancy between their existing state and the state experienced
  #...in the goal pursuit episode
  
  #Note that this learning rule does not incorporate interactions between 
  #...cues as canonical associative learning models do, but still are
  #...able to model simple inhibitory phenomena
  
  PE <- (1 - abs(V)) * abs(f_inc)     #Prediction error for proxy
  PEnf <- (1 - abs(Vnf)) * abs(f_inc_nf)  #Prediction error for negative feature
  
  #Store and update associative values
  V_evol <- c(V_evol, V)
  V <- V + PE * lr_cue * sign(f_inc)
  
  Vnf_evol <- c(Vnf_evol, Vnf)
  Vnf <- Vnf + PEnf * lr_nf * sign(f_inc_nf)
  
  #Initialize position tracking
  previous_x <- current_x
  previous_y <- current_y

  #Initialize vector storing feedback paths
  feedback_path <- c()      #Store proxy cue encounters
  feedback_path_nf <- c()   #Store negative feature encounters
  
  #Set step counter to zero
  k <- 0
  
  #Displacement Loop: Continue until goal is reached
  while (!path_cross_goal(previous_x, previous_y, current_x, current_y, goal_x, goal_y, goal_radius)) {
    #Add unity to step counter
    k <- k + 1
    #Update position tracking
    previous_x <- current_x
    previous_y <- current_y
    
    #Count proxy cues at current position
    inside_blobs <- sapply(1:num_blobs, function(i) {
      point_in_polygon(current_x, current_y, blobs[[i]])
    })
    inside_blob_count <- sum(inside_blobs)
    
    #Count negative feature cues at current position
    inside_nf <- sapply(31:num_blobs, function(i) {
      point_in_polygon(current_x, current_y, blobs[[i]])
    })
    inside_nf_count <- sum(inside_nf)
    
    #Store cue encounters and calculate changes in environmental information
    feedback_path <- c(feedback_path, inside_blob_count)
    gain <- (inside_blob_count - prev_cue)
    
    feedback_path_nf <- c(feedback_path_nf, inside_nf_count)
    gain_nf <- (inside_nf_count - prev_nf)
    
    #Calculate combined gain and determine movement parameters
    gain_comp <- gain * V + gain_nf * Vnf    #Combined gain from all cues
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
    
    if(i == loops){
      #Record current step data only in the last loop to improve execution time
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
        proxy_cue = inside_blob_count,
        gain_cue = gain,
        neg_feat = inside_nf_count,
        gain_feat = gain_nf,
        aff = case_when(
          aff == 0 ~ "app",
          aff == pi ~ "av",
          TRUE ~ as.character(aff)
        ),
        gain_tot = gain_f
      )
      
      #Update step record
      steps <- bind_rows(steps, new_step)
    }
    
    #Update tracking variables for next iteration
    prev_cue <- inside_blob_count
    prev_nf <- inside_nf_count 
  }
  
  #Post-episode Processing ----
  
  #Calculate proxy cue acceleration
  gain_acceleration <- diff(c(feedback_path, 30))
  
  gain_acceleration <- gain_acceleration/max(abs(gain_acceleration))
  n <- length(gain_acceleration)
  weights <- rev(0.5 / 2^(0:(n-1)))  #Exponentially decaying weights
  f_inc <- sum(gain_acceleration * weights) / sum(weights)
  
  #Calculate negative feature acceleration
  gain_acceleration_nf <- diff(c(feedback_path_nf, 0))
  
  #Ensure all values in gain_acceleration vector are within [-1, 1] by 
  #...dividing by the maximum absolute value
  #Check that not all values are zero to avoid dividing by zero 
  #...when the agent has avoided all deceiving cues.
  if (!all(gain_acceleration_nf == 0)) { 
    gain_acceleration_nf <- gain_acceleration_nf / max(abs(gain_acceleration_nf))
  }
  n_nf <- length(gain_acceleration_nf)
  weights_nf <- rev(0.5 / 2^(0:(n_nf-1)))
  f_inc_nf <- sum(gain_acceleration_nf * weights_nf) / sum(weights_nf)
  
  #Store episode performance metric
  steps_count_per_iteration[i] <- k
}

#Adjust the outcome data frame to enhance readability and facilitate inspection
steps$proxy_cue <- c(steps$proxy_cue[-1], 30)
steps$gain_cue <- c(steps$gain_cue[-1], NA)
steps$neg_feat <- c(steps$neg_feat[-1], 0)
steps$gain_feat <- c(steps$gain_feat[-1], NA)
steps$aff <- c(steps$aff[-1], NA)
steps$gain_tot <- c(steps$gain_tot[-1], NA)
steps$gain_tot <- format(steps$gain_tot, scientific = FALSE)

#Basic inspection of the output

#Visualize the temporal progression of steps per episode, which should be
#... fairly stationary
plot(steps_count_per_iteration, type = "l",
     main = "Progression of Performance Over Episodes",
     xlab = "Episode Number",
     ylab = "Steps to Goal",
     col = "steelblue")

#Calculate the typical performance metric (EARLY STAGE)
typical_performance_E <- median(steps_count_per_iteration[1:10])
print(paste("Typical steps per episode:", typical_performance_E))

#Calculate the typical performance metric (LATE STAGE)
typical_performance_L <- median(steps_count_per_iteration[991:1000])
print(paste("Typical steps per episode:", typical_performance_L))

#Find an episode that represents typical performance at the late stage of learning
#If a value matches the median, select it as representative; otherwise, 
#...select the highest value lower than the median
late_episodes <- steps_count_per_iteration[991:1000]
representative_episode_L <- 
  which(late_episodes == ifelse(any(late_episodes == typical_performance_L), 
                                typical_performance_L, 
                                max(late_episodes[late_episodes < typical_performance_L])))
print(paste("Representative episode number, late stage:", 
            representative_episode_L+990,
            "| Numer of steps:",
            steps_count_per_iteration[representative_episode_L+990]))

plot(V_evol, 
     main = "Progression of Associative Value Over Episodes", 
     xlab = "Associative Value", ylab = "Episode Number",
     col = "goldenrod")

plot(Vnf_evol, 
     main = "Progression of Associative Value Over Episodes", 
     xlab = "Associative Value", ylab = "Episode Number",
     col = "goldenrod")

#Plot the agent's path during the last episode
plot(c(0, steps$x), c(0, steps$y), type = "l",
     main = "Agent Trajectory in the Last Episode",
     xlab = "X Position",
     ylab = "Y Position",
     col = "darkgreen")

#Store the vector containing the distribution of steps per episode
steps_inhibition <- steps_count_per_iteration
V_evol_inhibition <- V_evol
Vnf_evol_inhibition <- Vnf_evol

#-----------------------------------------------------------------------------
# VISUALIZATION
#-----------------------------------------------------------------------------

#Early stage

#To generate the trajectory plot shown in Figure 3:
#Set loops <- 995 to capture the representative episode of proxy failure
#Execute until the simulation loop (stopping before the analysis section)
#Run the command below to store the agent's path

rep_episode_inhibition <- steps

#Prepare data for plotting
blob_plot_data <- do.call(rbind, lapply(1:num_blobs, function(i) {
  blob <- blobs[[i]]
  blob$id <- num_blobs - i + 1  #Reverse the id numbering
  blob
}))

#Clip blobs to the visible region [0, 10]
blob_plot_data_inhibition <- blob_plot_data %>%
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
  proxy_cue = NA,     
  gain_cue = NA,
  neg_feat = NA,
  gain_feat = NA,
  aff = NA,      
  gain_tot = NA      
)

#Combine the new row with existing data
rep_episode_inhibition <- bind_rows(new_row, rep_episode_inhibition)

agent_path_inhibition <- ggplot(blob_plot_data_inhibition, aes(x = x, y = y, group = id)) +
  geom_polygon(aes(fill = id <= 60), alpha = 0.04, color = "gray95") +  # Changed to <= and simplified the factor conversion
  geom_circle(aes(x0 = goal_x, y0 = goal_y, r = goal_radius), fill = "red4", alpha = 1) +
  scale_fill_manual(
    values = c("magenta", "blue"),  #First value is for FALSE (>60), second for TRUE (<=60)
    labels = c("Proxy", "Proxy + Negative Feature"),
    guide = guide_legend(reverse = FALSE, override.aes = list(alpha = 1))
  ) +
  geom_path(data = rep_episode_inhibition, aes(x = x, y = y, group = 1), 
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

print(agent_path_inhibition)
agent_path_inhibition <- agent_path_inhibition +
  labs(title = "Behavioral Inhibition") +
  theme(
    plot.title = element_text(
      size = 16,
      face = "bold",
      hjust = 0.5
    )
  )


#Main plot of steps for goal attainment
sinh <- ggplot(data = data.frame(Episodes = seq_along(steps_inhibition),
                                Steps = steps_inhibition), 
              aes(x = Episodes, y = Steps)) +
  geom_line() +
  labs(x = "Episodes",
       y = "Number of Steps to Attain Goal") +
  scale_x_continuous(breaks = c(c(0:10)*100)) +
  scale_y_continuous(breaks = c(c(c(0:12)*100))) +
  theme_minimal() +
  theme(plot.margin = margin(5.5, 40, 5.5, 5.5),
        panel.grid = element_blank(),
        panel.border = element_rect(color = "black", fill = NA),
        axis.text.x = element_text(size = 13),
        axis.text.y = element_text(size = 12),
        axis.title.x = element_text(size = 16),
        axis.title.y = element_text(size = 16)    
  )

#Inset plot of associative strength progression
vinh <-  ggplot() +  
  scale_y_continuous(limits = c(-1, 1)) +
  geom_hline(yintercept = 0, color = "black") +
  geom_vline(xintercept = 0, color = "black") +
  geom_point(data = data.frame(Episodes = seq_along(V_evol_inhibition),
                               Value = V_evol_inhibition,
                               Type = "V_evol_inhibition"), 
             aes(x = Episodes, y = Value), 
             color = "magenta", alpha = 0.1, size = 2.5) +
  geom_point(data = data.frame(Episodes = seq_along(Vnf_evol_inhibition),
                               Value = Vnf_evol_inhibition,
                               Type = "Vnf_evol_inhibition"), 
             aes(x = Episodes, y = Value), 
             color = "blue", alpha = 0.1, size = 2.5) +
  labs(x = "Episodes",
       y = "Associative Value") +
  theme_minimal(base_size = 8) +
  theme(plot.background = element_rect(fill = "white", color = NA),
        panel.grid = element_blank(),
        axis.text.x = element_text(size = 7),    
        axis.text.y = element_text(size = 8),
        axis.title.x = element_text(size = 11),   
        axis.title.y = element_text(size = 11)    
  )

#Combine plots with inset
sNv_inh <- sinh + inset_element(vinh, left = 0.75, bottom = 0.6, right = 0.85, top = 0.9)

print(sNv_inh)

#Ensamble plots for Figure 3
F3_top <- agent_path_pt + agent_path_inhibition
print(F3_top)
#Recommended dimensions: 1600 × 700 pixels

F3_bottom <- sNv_pt + sNv_inh + plot_layout(widths = c(1, 5))
print(F3_bottom)
#Recommended dimensions: 2000 × 500 pixels

#Finished!