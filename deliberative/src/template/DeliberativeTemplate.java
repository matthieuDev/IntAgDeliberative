package template;

/* import table */
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.LinkedList;

import logist.simulation.Vehicle;
import logist.agent.Agent;
import logist.behavior.DeliberativeBehavior;
import logist.plan.Action;
import logist.plan.Plan;
import logist.task.Task;
import logist.task.TaskDistribution;
import logist.task.TaskSet;
import logist.topology.Topology;
import logist.topology.Topology.City;

/**
 * An optimal planner for one vehicle.
 */
@SuppressWarnings("unused")
public class DeliberativeTemplate implements DeliberativeBehavior {
	
	public static void main(String[] args) {
		logist.LogistPlatform.main(args);
	}

	enum Algorithm { BFS, ASTAR }
	
	/* Environment */
	Topology topology;
	TaskDistribution td;
	
	/* the properties of the agent */
	Agent agent;
	int capacity;
	

	/* the planning class */
	Algorithm algorithm;
	
	int nbCity ;
	ArrayList<Task> taskList = new ArrayList<Task>();
	City planCity ;
	Comparator<State> comparatorState ;
	
	@Override
	public void setup(Topology topology, TaskDistribution td, Agent agent) {
		this.topology = topology;
		this.td = td;
		this.agent = agent;
		
		// initialize the planner
		int capacity = agent.vehicles().get(0).capacity();
		String algorithmName = agent.readProperty("algorithm", String.class, "ASTAR");
		
		// Throws IllegalArgumentException if algorithm is unknown
		algorithm = Algorithm.valueOf(algorithmName.toUpperCase());
		
		// ...
		nbCity = topology.cities().size();
		comparatorState = new Comparator<State>() {
			@Override
	        public int compare(State o1, State o2) {
	            if (o1.cost < o2.cost ) {
	            	return -1 ;
	            } else if ( o1.cost == o2.cost) {
	            	return 0 ;
	            } else {
	            	return 1;
	            }
	        }
		};
	}
	
	@Override
	public Plan plan(Vehicle vehicle, TaskSet tasks) {
		Plan plan;
		createTaskList(tasks);

		// Compute the plan with the selected algorithm.
		switch (algorithm) {
		case ASTAR:
			// ...
			plan = AStarPlan( vehicle);
			break;
		case BFS:
			// ...
			plan = AStarPlan( vehicle);
			break;
		default:
			throw new AssertionError("Should not happen.");
		}	
		for( Action t : plan) {
			System.out.println(t);
		}
		return plan;
	}
	
	private Plan naivePlan(Vehicle vehicle, TaskSet tasks) {
		City current = vehicle.getCurrentCity();
		Plan plan = new Plan(current);

		for (Task task : tasks) {
			// move: current city => pickup location
			for (City city : current.pathTo(task.pickupCity))
				plan.appendMove(city);

			plan.appendPickup(task);

			// move: pickup location => delivery location
			for (City city : task.path())
				plan.appendMove(city);

			plan.appendDelivery(task);

			// set current city
			current = task.deliveryCity;
		}
		return plan;
	}
	
	private Plan AStarPlan(Vehicle vehicle) {
		LinkedList<State> Q = new LinkedList<State>() , C = new LinkedList<State>();
		ArrayList<Integer> tasks =  new ArrayList<Integer>() , tasksTaken =  new ArrayList<Integer>();
		
		for (int i = 0; i < taskList.size(); i++) {
			tasks.add(i);
		}
		
		planCity=vehicle.getCurrentCity();
		Q.add(new State( new Plan(planCity), vehicle.homeCity(),tasks , tasksTaken));
		Plan res = null ;
		
		while ( res == null) {

			
			if (Q.isEmpty()) return res ;
			
			State n = Q.pop();
			
			
			
			if (n.isFinal()) res =n.plan;
			
			if( ! compareIn(C, n)) {
				C.push(n);
				
				int weightCarry = 0 ;
				for ( int t : n.taskTaken) {
					weightCarry += taskList.get(t).weight;
				}
				
				LinkedList<State> S = new LinkedList<State>();
				
				for ( City c : n.currentCity.neighbors()) {
					Plan plan = new Plan (planCity);
					for (Action a : n.plan) {
						plan.append(a);
					}
					plan.appendMove(c);
					tasksTaken = (ArrayList<Integer>) n.taskTaken.clone();
					ArrayList<Integer> taskFree = (ArrayList<Integer>) n.taskFree.clone();
					
					for(int t : n.taskFree) {
						Task task = taskList.get(t);
						if ( weightCarry + task.weight <= vehicle.capacity() && task.pickupCity.equals(c)){
							
							plan.appendPickup(task);
							
							ArrayList<Integer> newTaskFree = new ArrayList<Integer>();
							for( int t2 : taskFree) {
								if(t2 != t) newTaskFree.add(t2);
							}
							taskFree = newTaskFree ;
							tasksTaken.add(t);
							
						}
					}
					
					for ( int t : n.taskTaken) {
						Task task = taskList.get(t);
						if (task.deliveryCity.equals(c)){
							plan.appendDelivery(task);
							
							ArrayList<Integer> newTaskTaken = new ArrayList<Integer>();
							for( int t2 : tasksTaken) {
								if(t2 != t) newTaskTaken.add(t2);
							}
							tasksTaken = newTaskTaken ;
						}
					}
					State newState = new State(plan, c , taskFree, tasksTaken);
					newState.cost = (int) (n.cost + n.currentCity.distanceTo(c)*vehicle.costPerKm());
					 S.add(newState);
				}
				
				
				Collections.sort(S, comparatorState);
				Q = mergeSort(Q, S);
				
			}
		}
		
		
		return res;
	}

	
	
	private Plan BFSPlan(Vehicle vehicle) {
		LinkedList<State> Q = new LinkedList<State>() , C = new LinkedList<State>();
		ArrayList<Integer> tasks =  new ArrayList<Integer>() , tasksTaken =  new ArrayList<Integer>();
		
		for (int i = 0; i < taskList.size(); i++) {
			tasks.add(i);
		}
		
		planCity=vehicle.getCurrentCity();
		Q.add(new State( new Plan(planCity), vehicle.homeCity(),tasks , tasksTaken));
		Plan res = null ;
		
		while ( res == null) {

			
			if (Q.isEmpty()) return res ;
			
			State n = Q.pop();
			
			
			
			if (n.isFinal()) res =n.plan;
			
			if( ! contains(C, n)) {
				C.push(n);
				
				int weightCarry = 0 ;
				for ( int t : n.taskTaken) {
					weightCarry += taskList.get(t).weight;
				}
				
				for ( City c : n.currentCity.neighbors()) {
					Plan plan = new Plan (planCity);
					for (Action a : n.plan) {
						plan.append(a);
					}
					plan.appendMove(c);
					tasksTaken = (ArrayList<Integer>) n.taskTaken.clone();
					ArrayList<Integer> taskFree = (ArrayList<Integer>) n.taskFree.clone();
					
					for(int t : n.taskFree) {
						Task task = taskList.get(t);
						if ( weightCarry + task.weight <= vehicle.capacity() && task.pickupCity.equals(c)){
							
							plan.appendPickup(task);
							
							ArrayList<Integer> newTaskFree = new ArrayList<Integer>();
							for( int t2 : taskFree) {
								if(t2 != t) newTaskFree.add(t2);
							}
							taskFree = newTaskFree ;
							tasksTaken.add(t);
							
						}
					}
					
					for ( int t : n.taskTaken) {
						Task task = taskList.get(t);
						if (task.deliveryCity.equals(c)){
							plan.appendDelivery(task);
							
							ArrayList<Integer> newTaskTaken = new ArrayList<Integer>();
							for( int t2 : tasksTaken) {
								if(t2 != t) newTaskTaken.add(t2);
							}
							tasksTaken = newTaskTaken ;
						}
					}
					State newState = new State(plan, c , taskFree, tasksTaken);
					 Q.add(newState);
				}
				
				
				
				
			}
		}
		
		
		return res;
	}

	@Override
	public void planCancelled(TaskSet carriedTasks) {
		
		if (!carriedTasks.isEmpty()) {
			// This cannot happen for this simple agent, but typically
			// you will need to consider the carriedTasks when the next
			// plan is computed.
		}
	}
	
	public void createTaskList( TaskSet tasks) {
		for(Task t : tasks ) {
			taskList.add(t);
		}
	}
	
	private boolean contains ( LinkedList<State> list , State state ) {
		for ( State s : list){
			if (s.equals(state)) return true ;
		}
		return false;
	}
	
	private boolean compareIn ( LinkedList<State> list , State state ) {
		for ( State s : list){
			if (s.equals(state) ) {
				if (s.cost<=state.cost) {
					return true ;
				} else {
					list.remove(s);
					return false;
				}
				
			} 
		}
		return false;
	}
	
	private LinkedList<State> mergeSort (LinkedList<State> l1 , LinkedList<State> l2  ){
		LinkedList<State> sort = new LinkedList<State>();
		while(! (l1.isEmpty() && l2.isEmpty() ) ) {
			if (l1.isEmpty()) {
				sort.add(l2.pop());
			}
			else if (l2.isEmpty()) {
				sort.add(l1.pop());
			} else {
				State s1 = l1.get(0) , s2 = l2.get(0);
				if ( s1.cost < s2.cost ) {
					sort.add(s1);
					l1.pop();
				} else {
					sort.add(s2);
					l2.pop();
				}
			}
		}
		return sort ;
	}
	
	private class State {
		public City currentCity ;
		public ArrayList<Integer> taskFree, taskTaken ;
		public Plan plan  ;
		public int cost = 0;
		@SuppressWarnings("unchecked")
		public State (Plan ts ,  City c , ArrayList<Integer> tf , ArrayList<Integer> tt ) {
			currentCity = c ;
			taskFree = (ArrayList<Integer>)tf.clone();
			taskTaken = (ArrayList<Integer>)tt.clone();
			plan = ts;
		}
		
		public boolean equals ( State s ) {
			if ( ! (s.taskFree.size() == taskFree.size() &&
					s.taskTaken.size() == taskTaken.size() )) return false ;
			for(Integer t : taskFree){
				if ( ! s.taskFree.contains(t)) return false ;
			}
			for (Integer t : taskTaken ) {
				if ( ! s.taskTaken.contains(t)) return false ;
			}
			return s.currentCity.equals(currentCity) ;
		}
		
		public boolean isFinal () { return taskFree.isEmpty() && taskTaken.isEmpty();}
		}
}
