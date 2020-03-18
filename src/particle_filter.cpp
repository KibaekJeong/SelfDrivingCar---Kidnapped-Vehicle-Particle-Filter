/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * Add random Gaussian noise to each particle.

    num_particles = 50;  // TODO: Set the number of particles
    default_random_engine gen;
    normal_distribution<double> dist_x(x,std[0]);
    normal_distribution<double> dist_y(y,std[1]);
    normal_distribution<double> dist_theta(theta,std[2]);
    for(int i=0;i<num_particles;i++){
        Particle p;
        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1.;
        particles.push_back(p);
        weights.push_back(p.weight);
    }
    is_initialized = true;
    cout<<"Initialized\n";


}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * Add measurements to each particle and add random Gaussian noise.
   *
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
    default_random_engine gen;

    for (int i = 0; i<num_particles;i++){
        double p_x = particles[i].x;
        double p_y = particles[i].y;
        double p_theta = particles[i].theta;
        double pred_x;
        double pred_y;
        double pred_theta;
        if(fabs(yaw_rate)<0.0001){
            pred_x = p_x + (velocity*cos(p_theta)*delta_t);
            pred_y = p_y + (velocity*sin(p_theta)*delta_t);
            pred_theta = p_theta;
        }
        else{
            pred_x = p_x + (velocity/yaw_rate)*(sin(p_theta+(yaw_rate*delta_t))-sin(p_theta));
            pred_y = p_y + (velocity/yaw_rate)*(cos(p_theta)-cos(p_theta+(yaw_rate*delta_t)));
            pred_theta = p_theta + yaw_rate*delta_t;
        }
        normal_distribution<double> dist_x(pred_x,std_pos[0]);
        normal_distribution<double> dist_y(pred_y,std_pos[1]);
        normal_distribution<double> dist_theta(pred_theta,std_pos[2]);
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
    }


}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
  /**
   * Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   */
    for (int i=0;i<observations.size();i++){
        double min_dist = numeric_limits<double>::max();
        for(int j=0;j<predicted.size();j++){
            double distance = dist(observations[i].x,observations[i].y,predicted[j].x,predicted[j].y);
            if(distance<min_dist){
                min_dist = distance;
                observations[i].id = predicted[j].id;
            }
        }
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /**
   * Update the weights of each particle using a mult-variate Gaussian
   *   distribution. You can read more about this distribution here:
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution

   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
    double weight_sum = 0.0;

    for (int i=0;i<num_particles;i++){
        double p_x = particles[i].x;
        double p_y = particles[i].y;
        double p_theta = particles[i].theta;
        vector<LandmarkObs>transformed_observation;
        // Transform from vehicle coordinates to map coordinates system
        for (int j=0;j<observations.size();j++){
            LandmarkObs transformed_ob;
            transformed_ob.id = j;
            transformed_ob.x = p_x + (observations[j].x*cos(p_theta)) - (sin(p_theta)*observations[j].y);
            transformed_ob.y = p_y + (observations[j].x*sin(p_theta))+(cos(p_theta)*observations[j].y);
            transformed_observation.push_back(transformed_ob);
        }
        // check range and get landmarks in sensor range
        vector<LandmarkObs> predicted_landmarks;
        for (int k=0;k<map_landmarks.landmark_list.size();k++){
            double distance = dist(map_landmarks.landmark_list[k].x_f,map_landmarks.landmark_list[k].y_f,p_x,p_y);
            if (distance<sensor_range){
                LandmarkObs pred;
                pred.id = map_landmarks.landmark_list[k].id_i;
                pred.x = map_landmarks.landmark_list[k].x_f;
                pred.y = map_landmarks.landmark_list[k].y_f;
                predicted_landmarks.push_back(pred);
            }
        }
        // data association
        dataAssociation(predicted_landmarks,transformed_observation);

        double sig_x = std_landmark[0];
        double sig_y = std_landmark[1];
        double sig_x_2 = pow(sig_x,2);
        double sig_y_2 = pow(sig_y,2);
        double norm = 1.0/(2*M_PI*sig_x*sig_y);
        double w = 1.0;
        for(int l=0; l<transformed_observation.size();l++){
            double t_x = transformed_observation[l].x;
            double t_y = transformed_observation[l].y;
            double t_id = transformed_observation[l].id;
            Map::single_landmark_s lm = map_landmarks.landmark_list.at(t_id-1);
            w *= (norm* exp(-1.*((pow(t_x-lm.x_f,2)/(2*sig_x_2))+(pow(t_y-lm.y_f,2)/(2*sig_y_2)))));
        }
        //update weight
        particles[i].weight = w;

        weight_sum += w;

    }
    for(int m=0;m<particles.size();m++){
        particles[m].weight /= weight_sum;
        weights[m] = particles[m].weight;
    }
}

void ParticleFilter::resample() {
  /**
   * Resample particles with replacement with probability proportional
   *   to their weight.
   */
    default_random_engine gen;
    discrete_distribution<>distribution_weighted(weights.begin(),weights.end());
    vector<Particle> resampled;
    for(int i=0;i<particles.size();i++){
        int id = distribution_weighted(gen);
        resampled.push_back(particles[id]);
    }
    particles = resampled;
}

void ParticleFilter::SetAssociations(Particle& particle,
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
