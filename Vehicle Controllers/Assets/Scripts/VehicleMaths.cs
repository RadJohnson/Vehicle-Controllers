namespace RADVehichles
{
    internal struct VehicleMaths
    {
        /// <summary>
        /// Calcualtes Hookes Law
        /// </summary>
        /// <param name="K">is the Spring Constant(stiffnes)</param>
        /// <param name="X">is the Compression</param>
        /// <returns> </returns>
        public static float HookesLawCompressionSpring(float K, float X)
        {
            float F = K * X;//need negative K for when I want an extension spring not when I want compression spring
            return F;
        }

        /// <summary>
        /// Calcualtes Hookes Law
        /// </summary>
        /// <param name="K">is the Spring Constant(stiffnes)already negative in the function</param>
        /// <param name="X">is the Extension</param>
        /// <returns> </returns>
        public static float HookesLawExtensionSpring(float K, float X)
        {
            float F = -K * X;
            return F;
        }

        /// <summary>
        /// Used to calculate the speed the suspension is moving so that the damping can counteract it correctly
        /// </summary>
        /// <param name="lasPos"></param>
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
    }
}