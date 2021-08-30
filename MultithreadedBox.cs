using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System;
using Unity.Jobs;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;




public struct PICSimulationGlobals {
    
    public float wait_time;

    public int num_of_particles;
    public float length;
    public float width;
    public float height;

    public int num_of_cells;

    public float q;
    public float mass_e;
    public float epsi0;
    public float dx;
    public float dt;
    public int solver_iter;
    public float res_tolerance;
    public float scale_factor;
    public float len;
    public float qm;
    [NativeDisableContainerSafetyRestriction]
    public NativeArray<Vector3> position;

    [NativeDisableContainerSafetyRestriction]
    public NativeArray<Vector3> velocity;

    [NativeDisableContainerSafetyRestriction]
    public NativeArray<float> rho;

    [NativeDisableContainerSafetyRestriction]
    public NativeArray<float> phi;

    [NativeDisableContainerSafetyRestriction]
    public NativeArray<float> E_fields;

    public void GetLeftAndRightIndexes(ref int left, ref int right, int index) {
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
}




public struct PICSimulationJobData {
    public int start_index;
    public int end_index;
    public PICSimulationGlobals globals;
}

public struct ClearCell : IJob {
    [NativeDisableContainerSafetyRestriction]
    public PICSimulationJobData data;

    public void Execute() {
        
        PICSimulationGlobals globals = data.globals;
        int start_index = data.start_index;
        int end_index = data.end_index;

        for (int i = start_index; i < end_index; i++) {
            globals.rho[i] = 0;
            globals.phi[i] = 0;
        }
    }
}

public struct AssignParticleToCell: IJob {
    [NativeDisableContainerSafetyRestriction]
    public PICSimulationJobData data;

    public void Execute() {
        
        PICSimulationGlobals globals = data.globals;
        int start_index = data.start_index;
        int end_index = data.end_index;

        for (int i = start_index; i < end_index; i++) {
            int left = (int) Math.Floor(globals.position[i].x / globals.dx);

            float d = globals.position[i].x / globals.dx - left;
            
            //Debug.Log(i + "th left: " + left + ", " + i + "th position: " + globals.particles[i].position.x);
            globals.rho[left] += globals.q * (1 - d);
            globals.rho[left + 1] += globals.q * d;
        }
    }
}

public struct CalculateRho: IJob {
    [NativeDisableContainerSafetyRestriction]
    public PICSimulationJobData data;

    public void Execute() {

        PICSimulationGlobals globals = data.globals;
        int start_index = data.start_index;
        int end_index = data.end_index;

        for (int i = start_index; i < end_index; i++) {
            globals.rho[i] /= globals.dx;
        }

        //background charges???
        if (end_index == globals.num_of_cells - 1) {
            end_index -= 1;

        }
        for (int i = start_index; i < end_index; i++) {
            globals.rho[i] += (-globals.q) * globals.num_of_particles / (globals.num_of_cells - 1);
        }
    }
}

public struct CalculatePhi: IJob {
    [NativeDisableContainerSafetyRestriction]
    public PICSimulationJobData data;

    public void Execute() {

        PICSimulationGlobals globals = data.globals;
        int start_index = data.start_index;
        int end_index = data.end_index;

        if (end_index == globals.num_of_cells - 1) {
            end_index -= 1;
        }

        for (int j = start_index; j < end_index; j++) {
            int left = 0;
            int right = 0;
            globals.GetLeftAndRightIndexes(ref left, ref right, j);
            //gs solver
            float new_potential = (float) 0.5 * (globals.phi[left] + globals.phi[right] + globals.dx * globals.dx * globals.rho[j] / globals.epsi0);
            //experiment with new weight!!!
            globals.phi[j] = (float) (globals.phi[j] + 1.4 * (new_potential - globals.phi[j]));
        }
    }
}

public struct CalculateEField: IJob {
    [NativeDisableContainerSafetyRestriction]
    public PICSimulationJobData data;

    public void Execute() {

        PICSimulationGlobals globals = data.globals;
        int start_index = data.start_index;
        int end_index = data.end_index;

        for (int i = start_index; i < end_index; i++) {
            int left = 0;
            int right = 0;
            globals.GetLeftAndRightIndexes(ref left, ref right, i);
            float E = (globals.phi[right] - globals.phi[left]) / (2 * globals.dx);
            globals.E_fields[i] = E;
        }
    }
}

public struct CalculateParticleTrajectory: IJob {
    [NativeDisableContainerSafetyRestriction]

    public PICSimulationJobData data;

    public void Execute() {

        PICSimulationGlobals globals = data.globals;
        int start_index = data.start_index;
        int end_index = data.end_index;

        //calculate e fields and acceleration
        for (int i = start_index; i < end_index; i++) {
            Vector3 prev_position = globals.position[i];
            Vector3 prev_velocity = globals.velocity[i];

            int left = (int) Math.Floor(prev_position.x / globals.dx);

            float d = prev_position.x / globals.dx - left;
            float E = globals.E_fields[left] * (1 - d) + globals.E_fields[left + 1] * d;


            //Debug.Log("electric field on " + i + "th particle: " + E);
            float acceleration = - E * globals.q / globals.mass_e * globals.scale_factor;
        
            //Debug.Log("acceleration of " + i + "th particle: " + acceleration);
            Vector3 velocity = new Vector3(prev_velocity.x + acceleration * globals.dt, 0, 0);
            //Debug.Log("velocity of " + i + "th particle: " + velocity.ToString());
            Vector3 position = new Vector3(prev_position.x + velocity.x * globals.dt, 0 ,0);
            //Debug.Log("position of " + i + "th particle: " + position.ToString());


            //doesn't this make the simulation inaccurate???
            if (position.x > globals.len) {
                //position.x = position.x % globals.len;
                position.x -= globals.len;
            }
            if (position.x < 0) {
                //Debug.Log(i + "th particle has <0 position after " + iter_counter + "th iteration. It is " + position.x);
                //position.x = UnityEngine.Random.Range(0, globals.len);
                position.x += globals.len;
            }
            globals.position[i] = position;
            globals.velocity[i] = velocity;
        
            //Debug.Log("after " + iter_counter + "th iteration, " + "xv coordinates of " + i + "th particle: " + temp_object.transform.position.ToString());
        }
    }
}

public class MultithreadedBox : MonoBehaviour
{
    
    private int num_of_jobs = 8;

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

    private double start_time;
    private double prev_time;

    private List<GameObject> particles = new List<GameObject>();
  
    
    PICSimulationGlobals globals = new PICSimulationGlobals();



    void OnMouseDown() {
        Debug.Log("box clicked");
    }


    void GenerateParticles() 
    {
        var position = new NativeArray<Vector3>(num_of_particles, Allocator.Persistent);
        var velocity = new NativeArray<Vector3>(num_of_particles, Allocator.Persistent);

        for (int i = 0; i < num_of_particles; i++) {
            Vector3 particle_position = new Vector3(UnityEngine.Random.Range(0.0f, len), 0, 0);
            Vector3 particle_velocity;

            if (i % 2 == 0) {
                particle_velocity = new Vector3(1, 0, 0);
            }
            else {
                particle_velocity = new Vector3(-1, 0, 0);
            }
            Debug.Log("ready to instantiate particles!");
            GameObject particle_instance = Instantiate(particle, new Vector3(particle_position.x, particle_velocity.x, 0), Quaternion.identity);
            Debug.Log("instatiated particle!");

            if (i % 2 == 0) {
                var renderer = particle_instance.GetComponent<Renderer>();
                renderer.material.SetColor("_Color", Color.red);
            }
            else {
                var renderer = particle_instance.GetComponent<Renderer>();
                renderer.material.SetColor("_Color", Color.blue);
            }

            particles.Add(particle_instance);
            position[i] = new Vector3(UnityEngine.Random.Range(0.0f, len), 0, 0);
            velocity[i] = new Vector3(1, 0, 0);
            Debug.Log("finish assigning pos and vel");
        }

        var rho = new NativeArray<float>(num_of_cells, Allocator.Persistent);
        var phi = new NativeArray<float>(num_of_cells, Allocator.Persistent);
        var E_fields = new NativeArray<float>(num_of_cells, Allocator.Persistent);

        for (int i = 0; i < num_of_cells; i++) {
                rho[i] = 0.0f;
                phi[i] = 0.0f;
                E_fields[i] = 0.0f;
        }


        //PICSimulationGlobals globals = new PICSimulationGlobals();

        globals.position = position;
        globals.velocity = velocity;
  
        globals.rho = rho;
        globals.phi = phi;
        globals.E_fields = E_fields;

        globals.wait_time = 0.1f;

        globals.num_of_particles = 20000;

        globals.num_of_cells = 1000;

        globals.q = -1.6e-19f;
        globals.mass_e = 9.1e-31f;
        globals.epsi0 = 8.85e-12f;
        globals.dx = 0.2f;
        globals.dt = 0.1f;
        globals.solver_iter = 2000;
        globals.res_tolerance = 1e-6f;
        globals.scale_factor = 0.0001f;
        globals.len = 19.0f;
        globals.qm = -1.0f;

        Debug.Log("done with generating particles");
    }





    
    public void ScheduleAndCompleteJob(int id, int total, PICSimulationGlobals globals) {
        NativeList<JobHandle> job_handle_list = new NativeList<JobHandle>(Allocator.Temp);
        for (int i = 0; i < num_of_jobs; i++) {
            int start_index = i / num_of_jobs * total;
            int end_index = (i + 1) / num_of_jobs * total;
            PICSimulationJobData data = new PICSimulationJobData();
            data.start_index = start_index;
            data.end_index = end_index;
            data.globals = globals;
            switch (id) {
                case 0:
                    ClearCell clear_cell_job = new ClearCell();
                    clear_cell_job.data = data;
                    JobHandle clear_cell_job_handle = clear_cell_job.Schedule();
                    job_handle_list.Add(clear_cell_job_handle);
                    break;
                case 1:
                    AssignParticleToCell assign_particle_to_cell_job = new AssignParticleToCell();
                    assign_particle_to_cell_job.data = data;
                    JobHandle assign_particle_to_cell_job_handle = assign_particle_to_cell_job.Schedule();
                    job_handle_list.Add(assign_particle_to_cell_job_handle);
                    break;
                case 2:
                    CalculateRho calculate_rho_job = new CalculateRho();
                    calculate_rho_job.data = data;
                    JobHandle calculate_rho_job_handle = calculate_rho_job.Schedule();
                    job_handle_list.Add(calculate_rho_job_handle);

                    break;
                case 3:
                    CalculatePhi calculate_phi_job = new CalculatePhi();
                    calculate_phi_job.data = data;
                    JobHandle calculate_phi_job_handle = calculate_phi_job.Schedule();
                    job_handle_list.Add(calculate_phi_job_handle);
                    break;
                case 4:
                    CalculateEField calculate_E_field_job = new CalculateEField();
                    calculate_E_field_job.data = data;
                    JobHandle calculate_E_field_job_handle = calculate_E_field_job.Schedule();
                    job_handle_list.Add(calculate_E_field_job_handle);
                    break;
                case 5:
                    CalculateParticleTrajectory calculate_particle_trajectory_job = new CalculateParticleTrajectory();
                    calculate_particle_trajectory_job.data = data;
                    JobHandle calculate_particle_trajectory_job_handle = calculate_particle_trajectory_job.Schedule();
                    job_handle_list.Add(calculate_particle_trajectory_job_handle);
                    break;
            
            }
        }
        JobHandle.CompleteAll(job_handle_list);
        job_handle_list.Dispose();
    }
    



    IEnumerator Animate() {

        
        while (true) {
            yield return new WaitForSeconds(wait_time);

            Debug.Log(iter_counter + "th iteration");

            //PrintParticlesPosition(particles);

            //ClearCell job
            ScheduleAndCompleteJob(0, num_of_cells, globals);


            
            //AssignParticleToCell job
            ScheduleAndCompleteJob(1, num_of_particles, globals);

            //CalculateRho job
            ScheduleAndCompleteJob(2, num_of_cells, globals);
            //periodic bc???
            globals.rho[globals.num_of_cells - 1] += globals.rho[0];
            globals.rho[0] = globals.rho[globals.num_of_cells - 1];




            float error = 0;
            for (int i = 0; i < solver_iter; i++) {

                //CalculatePhi job
                ScheduleAndCompleteJob(3, num_of_cells, globals);
                /*
                for (int j = 0; j < num_of_cells - 1; j++) {
                    int left = 0;
                    int right = 0;
                    globals.GetLeftAndRightIndexes(ref left, ref right, j);
                    //gs solver
                    float new_potential = (float) 0.5 * (globals.phi[left] + globals.phi[right] + globals.dx * globals.dx * globals.rho[j] / globals.epsi0);
                    //experiment with new weight!!!
                    globals.phi[j] = (float) (globals.phi[j] + 1.4 * (new_potential - globals.phi[j]));
                }
                */

                if (i % 10 == 0) {
                    float sum = 0;
                    for (int k = 0; k < num_of_cells - 1; k++) {
                        int left = 0;
                        int right = 0;
                        globals.GetLeftAndRightIndexes(ref left, ref right, k);
                        float res = globals.rho[k] / globals.epsi0 + (globals.phi[left] + globals.phi[right] - 2 * globals.phi[k]) / (globals.dx * globals.dx);
                        sum += res * res;
                    }
                    error = (float) Math.Sqrt(sum / globals.num_of_cells);
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

            globals.phi[num_of_cells - 1] = globals.phi[0];


            //CalculateEField job
            ScheduleAndCompleteJob(4, num_of_cells, globals);
            /*
            for (int i = 0; i < num_of_cells - 1; i++) {
                int left = 0;
                int right = 0;
                globals.GetLeftAndRightIndexes(ref left, ref right, i);
                float E = (globals.phi[right] - globals.phi[left]) / (2 * globals.dx);
                globals.E_fields[i] = E;
            }
            */

            //CalculateParticleTrajectory job
            ScheduleAndCompleteJob(5, num_of_particles, globals);

            //animation in the scene
            for (int i = 0; i < num_of_particles; i++) {
                GameObject temp = particles[i];
                temp.transform.position = new Vector3(globals.position[i].x, globals.velocity[i].x, 0);
                particles[i] = temp;
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
        globals.position.Dispose();
        globals.velocity.Dispose();
        globals.rho.Dispose();
        globals.phi.Dispose();
        globals.E_fields.Dispose();
        
    }

    
    void Start()
    {
        start_time = Time.realtimeSinceStartup;
        prev_time = start_time;
        GenerateParticles();
        StartCoroutine(Animate());
    }
    

}



/*
void GenerateParticles() 
{
    for (int i = 0; i < num_of_particles; i++) {
        //Initial position
        Vector3 position = new Vector3(Random.Range(0, 10), 0, 0);
        Vector3 velocity;
        if (i % 2 == 0) {
            velocity = new Vector3(1, 0, 0);
        }
        else {
            velocity = new Vector3(-1, 0, 0);
        }
        GameObject particle_object = Instantiate(particle, (position.x, velocity.x, 0), Quaternion.identity);
        particles.Add(new ParticleInfo(particle_object, position));
        int index = Math.Floor(position.x);
        float d = position.x - index;
        globals.rho[index] += q * (1 - d);
        rho[index + 1] += q * d;
    }

    for (int i = 0; i < num_of_cells; i++) {
        rho[i] /= dx;
    }

    //background charges???

    //periodic bc???
    rho[num_of_cells - 1] += rho[0];
    rho[0] = rho[num_of_cells - 1];

    for (int i = 0; i < num_of_cells; i++) {
        phi[i] = 0;
    }

    for (int i = 0; i < solver_iter; i++) {
        for (int j = 0; j < num_of_cells; j++) {
            int left, right;
            globals.GetLeftAndRightIndexes(left, right, j);
            //gs solver
            float new_potential = 0.5 * (phi[left] + phi[right] + dx * dx * rho[j] / epsi0);
            //experiment with new weight!!!
            phi[j] = phi[j] + 1.4 * (new_potential - phi[j]);
        }

        if (i % 25 == 0) {
            float sum = 0;
            for (int k = 0; k < num_of_cells; k++) {
                int left, right;
                globals.GetLeftAndRightIndexes(left, right, k);
                float res = rho[j] / epsi0 + (phi[left] + phi[right] - 2 * phi[k]) / (dx * dx);
                sum += res * res;
            }
            if (sum < res_tolerance) {
                break;
            }
        }
    }

    //calculate e fields and acceleration
    for (int i = 0; i < num_of_particles; i++) {
        ParticleInfo temp = particles[i];
        int left = Math.Floor(temp.position.x);
        float E = (phi[i] + phi[i + 1]) / (2 * dx);
        float acceleration = E * q / m;
        Vector3 velocity = new Vector3(temp.velocity.x + acceleration, 0, 0);
        Vector3 position = new Vector3(temp.position.x + acceleration)
        temp.UpdatePosition(position);
        temp.UpdateVelocity(velocity);
        //xv->xy   
        GameObject temp_object = temp.particle;
        temp_object.transform.position = (position.x, velocity.x, 0);
    }



}
*/
