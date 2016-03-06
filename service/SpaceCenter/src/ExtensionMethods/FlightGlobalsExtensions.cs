using System;
using System.Linq;

namespace KRPC.SpaceCenter
{
    static class FlightGlobalsExtensions
    {
        public static global::Vessel GetVesselById (Guid id)
        {
            global::Vessel vessel = null;
            ValueCache.VesselById.TryGetValue (id, out vessel);
            if (vessel == null) {
                if (FlightGlobals.ActiveVessel.id == id)
                    vessel = FlightGlobals.ActiveVessel;
                else
                    vessel = FlightGlobals.Vessels.Where (v => v.id == id).FirstOrDefault ();
                if (vessel == null)
                    throw new ArgumentException ("No such vessel " + id);
                ValueCache.VesselById [id] = vessel;
            }
            return vessel;
        }

        public static global::Part GetPartById (uint id)
        {
            global::Part part = null;
            ValueCache.PartById.TryGetValue (id, out part);
            if (part == null) {
                part = FlightGlobals.FindPartByID (id);
                if (part == null)
                    throw new ArgumentException ("No such part " + id);
                ValueCache.PartById [id] = part;
            }
            return part;
        }
    }
}
