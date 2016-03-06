using System;
using System.Collections.Generic;

namespace KRPC.SpaceCenter
{
    /// <summary>
    /// Caches various expensive-to-compute values across RPCs that are executed
    /// within the same FixedUpdate. For example, a vessels center of mass does not change
    /// during a single call to FixedUpdate, so it can be cached.
    /// </summary>
    static class ValueCache
    {
        internal static IDictionary<Guid, global::Vessel> VesselById { get; private set; }

        internal static IDictionary<uint, global::Part> PartById { get; private set; }

        internal static void Init ()
        {
            VesselById = new Dictionary<Guid, global::Vessel> ();
            PartById = new Dictionary<uint, global::Part> ();
            KRPCServer.Context.Server.OnUpdateStarted += (s, e) => {
                VesselById.Clear ();
                PartById.Clear ();
            };
        }
    }
}
