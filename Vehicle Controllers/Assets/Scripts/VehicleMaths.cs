namespace RADVehichles
{

    //TODO : rectify Hooks Law so that only one formula is present with propper description

    internal struct VehicleMaths
    {
        /// <summary>
        /// Calcualtes Hookes Law
        /// </summary>
        /// <param name="K">is the Spring Constant(stiffnes)</param>
        /// <param name="X">is the Compression</param>
        /// <returns> </returns>
        public static float HookesLaw(float K, float X)
        {
            float F = K * X;//need negative K for when I want an extension spring not when I want compression spring
            return F;
        }

        //TODO : Fix Sumary definition of Accelerations
        /// <summary>
        /// Used to calculate the speed the suspension is moving so that the damping can counteract it correctly
        /// </summary>
        /// <param name="lastPos"></param>
        /// <param name="curentPos"></param>
        /// <param name="t"></param>
        /// <returns></returns>
        public static float Acceleration(float lastPos, float curentPos, float t)
        {
            float a = (lastPos - curentPos) / t;
            return a;
        }


        /// <summary>                                                                                   
        /// Calcualtes Force of Friction                                                                
        /// </summary>                                                                                  
        /// <param name="µ">is the Friction Coefficient(ammount of interaction between surfaces)</param>
        /// <param name="N">is the Normal Force(applied perpendicular to the surface contact)</param>   
        /// <returns></returns>                                                                         
        public static float Friction(float µ, float N)
        {
            float F = µ * N;
            return F;
        }

        /// <summary>
        /// Calculates force to be applied by wheels
        /// </summary>
        /// <param name="u">Unit vector for heading  could just be forward vector for vehicle?</param>
        /// <param name="EngineForce">Power of the engine</param>
        /// <returns></returns>
        float Traction(float u, float EngineForce)
        {
            float F = u * EngineForce;
            return F;
        }
    }
}