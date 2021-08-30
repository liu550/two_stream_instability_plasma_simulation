
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System;
using Unity.Jobs;
using Unity.Collections;


struct ParticleInfo {
    public GameObject particle;
    public Vector3 position { get; set; }
    public Vector3 velocity { get; set; }

    public void UpdatePosition(Vector3 new_position) {
        position = new_position;
    }

    public void UpdateVelocity(Vector3 new_velocity) {
        velocity = new_velocity;
    }
}

public class Box : MonoBehaviour
{

    private float wait_time = 0.1f;

    public GameObject particle;
    static private int num_of_particles = 20000;
    public float length;
    public float width;
    public float height;

    static private int num_of_cells = 1000;
    private int num_of_iters = 100;
    private int iter_counter = 0;

    private float q = -1.6e-19f;
    private float mass_e = 9.1e-31f;
    private float epsi0 = 8.85e-12f;
    private float dx = 0.2f;
    private float dt = 0.1f;
    private int solver_iter = 2000;
    private float res_tolerance = 1e-6f;
    private float scale_factor = 0.0001f;
    private float len = 19.0f;
    private float qm = -1.0f;

    private List<ParticleInfo> particles = new List<ParticleInfo>(num_of_particles);
    private List<float> rho = new List<float>(num_of_cells);
    private List<float> phi = new List<float>(num_of_cells);
    private List<float> E_fields = new List<float>(num_of_cells);

    private double start_time;
    private double prev_time;
    


    void GetLeftAndRightIndexes(ref int left, ref int right, int index) {
        left = index - 1;
        right = index + 1;
        if (left < 0) {
            left = num_of_cells - 2;
        }
        if (right > num_of_cells - 1) {
            right = 0;
        }
        //Debug.Log("index: " + index + ", left: " + left + ", right: " + right);
    }

    void GenerateParticles() 
    {
        for (int i = 0; i < num_of_particles; i++) {
            Vector3 position = new Vector3(UnityEngine.Random.Range(0.0f, len), 0, 0);
            Vector3 velocity;

            if (i % 2 == 0) {
                velocity = new Vector3(1, 0, 0);
            }
            else {
                velocity = new Vector3(-1, 0, 0);
            }

            GameObject particle_object = Instantiate(particle, new Vector3(position.x, velocity.x, 0), Quaternion.identity);

            if (i % 2 == 0) {
                var renderer = particle_object.GetComponent<Renderer>();
                renderer.material.SetColor("_Color", Color.red);
            }
            else {
                var renderer = particle_object.GetComponent<Renderer>();
                renderer.material.SetColor("_Color", Color.blue);
            }

            ParticleInfo temp = new ParticleInfo();
            temp.particle = particle_object;
            temp.position = position;
            temp.velocity = velocity;
            particles.Add(temp);
        }

        for (int i = 0; i < num_of_cells; i++) {
                rho.Add(0);
                phi.Add(0);
        }

        Debug.Log("done with generating particles");
    }

    void PrintList(List<float> list) {
        for (int i = 0; i < list.Count; i++) {
            Debug.Log(list[i] + ", ");
        }
    }

    void PrintParticlesPosition(List<ParticleInfo> particles) {
        for (int i = 0; i < particles.Count; i++) {
            Debug.Log("particle" + i + " transform: " + particles[i].particle.transform.position.ToString());
        }
    }



    IEnumerator Animate() {

        
        while (true) {
            yield return new WaitForSeconds(wait_time);

            Debug.Log(iter_counter + "th iteration");

            //PrintParticlesPosition(particles);

            //clear rho and phi
            for (int i = 0; i < num_of_cells; i++) {
                rho[i] = 0;
                phi[i] = 0;
            }

            for (int i = 0; i < num_of_particles; i++) {
                int left = (int) Math.Floor(particles[i].position.x / dx);
                float d = particles[i].position.x / dx - left;
                
                //Debug.Log(i + "th left: " + left + ", " + i + "th position: " + particles[i].position.x);
                rho[left] += q * (1 - d);
                rho[left + 1] += q * d;
            }


            
            //Debug.Log("phi for " + iter_counter + "th iteration is: " + PrintList(phi));

            for (int i = 0; i < num_of_cells; i++) {
                rho[i] /= dx;
            }

            //background charges???
            for (int i = 0; i < num_of_cells - 1; i++) {
                rho[i] += (-q) * num_of_particles / (num_of_cells - 1);
            }

            //periodic bc???
            rho[num_of_cells - 1] += rho[0];
            rho[0] = rho[num_of_cells - 1];

            //Debug.Log("rho for " + iter_counter + "th iteration is: ");
            //PrintList(rho);

            for (int i = 0; i < num_of_cells; i++) {
                phi[i] = 0;
            }

            float error = 0;
            for (int i = 0; i < solver_iter; i++) {
                for (int j = 0; j < num_of_cells - 1; j++) {
                    int left = 0;
                    int right = 0;
                    GetLeftAndRightIndexes(ref left, ref right, j);
                    //gs solver
                    float new_potential = (float) 0.5 * (phi[left] + phi[right] + dx * dx * rho[j] / epsi0);
                    //experiment with new weight!!!
                    phi[j] = (float) (phi[j] + 1.4 * (new_potential - phi[j]));
                }

                if (i % 10 == 0) {
                    float sum = 0;
                    for (int k = 0; k < num_of_cells - 1; k++) {
                        int left = 0;
                        int right = 0;
                        GetLeftAndRightIndexes(ref left, ref right, k);
                        float res = rho[k] / epsi0 + (phi[left] + phi[right] - 2 * phi[k]) / (dx * dx);
                        sum += res * res;
                    }
                    error = (float) Math.Sqrt(sum / num_of_cells);
                    if (sum < res_tolerance) {
                        Debug.Log("error: " + error + " sum: " + sum);
                        break;
                    }
                    if (error < res_tolerance) {
                        Debug.Log("error: " + error);
                        break;
                    }
                }
            }
            Debug.Log("error after for loop: " + error);

            phi[num_of_cells - 1] = phi[0];

            int neg_e = 0;
            int pos_e = 0;

            for (int i = 0; i < num_of_cells; i++) {
                int left = 0;
                int right = 0;
                GetLeftAndRightIndexes(ref left, ref right, i);
                float E = (phi[right] - phi[left]) / (2 * dx);
                E_fields.Add(E);
            }

            //calculate e fields and acceleration
            for (int i = 0; i < num_of_particles; i++) {
                ParticleInfo temp = particles[i];
                int left = (int) Math.Floor(temp.position.x / dx);
                float d = particles[i].position.x / dx - left;
                float E = E_fields[left] * (1 - d) + E_fields[left + 1] * d;

                if (E > 0) {
                    pos_e += 1;
                }
                else neg_e += 1;

                //Debug.Log("electric field on " + i + "th particle: " + E);
                float acceleration = - E * q / mass_e * scale_factor;
            
                //Debug.Log("acceleration of " + i + "th particle: " + acceleration);
                Vector3 velocity = new Vector3(temp.velocity.x + acceleration * dt, 0, 0);
                //Debug.Log("velocity of " + i + "th particle: " + velocity.ToString());
                Vector3 position = new Vector3(temp.position.x + velocity.x * dt, 0 ,0);
                //Debug.Log("position of " + i + "th particle: " + position.ToString());


                //doesn't this make the simulation inaccurate???
                if (position.x > len) {
                    //position.x = position.x % len;
                    position.x -= len;
                }
                if (position.x < 0) {
                    //Debug.Log(i + "th particle has <0 position after " + iter_counter + "th iteration. It is " + position.x);
                    //position.x = UnityEngine.Random.Range(0, len);
                    position.x += len;
                }
                temp.UpdatePosition(position);
                temp.UpdateVelocity(velocity);
                //xv->xy   
                GameObject temp_object = temp.particle;
                temp_object.transform.position = new Vector3(position.x, velocity.x, 0);
                particles[i] = temp;
                //Debug.Log("after " + iter_counter + "th iteration, " + "xv coordinates of " + i + "th particle: " + temp_object.transform.position.ToString());
            }
            //Debug.Log(iter_counter + "th iteration. " + "number of positive e fields: " + pos_e);
            //Debug.Log(iter_counter + "th iteration. " + "number of negative e fields: " + neg_e);
            iter_counter += 1;

            double curr_time = Time.realtimeSinceStartup;
            Debug.Log("Time needed per iterations: " + (curr_time - prev_time));
            prev_time = curr_time;

            if (iter_counter == num_of_iters) {
                double end_time = Time.realtimeSinceStartup;
                Debug.Log("Time needed for " + num_of_iters + " iterations: " + (end_time - start_time));
                break;
            }
        }
        
    }

    
    void Start()
    {
        start_time = Time.realtimeSinceStartup;
        prev_time = start_time;
        GenerateParticles();
        StartCoroutine(Animate());
    }
    
}




