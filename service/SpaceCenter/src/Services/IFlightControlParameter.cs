﻿namespace KRPC.SpaceCenter.Services
{
    public interface IFlightControlParameter
    {
        bool Enabled { get; }
        bool IsAutopilot { get; }

        /// <summary>
        /// If this FlightControlParameter tends to fight with SAS, then it should
        /// have this be true so kOS knows to check for that condition when this
        /// control is active.
        /// </summary>
        bool FightsWithSas { get; }

        uint ControlPartId { get; }
        void UpdateValue(object value);
        object GetValue();
        /// <summary>
        /// The vessel that was responsible for having set this control.
        /// </summary>
        /// <returns>The responsible vessel.</returns>
        Vessel GetResponsibleVessel();
        /// <summary>
        /// Called each update in which kOS CONFIG has autopilot enabled, to
        /// ask this controller how to set the controls.
        /// </summary>
        /// <param name="c">the flight control state it should try to alter</param>
        void UpdateAutopilot(FlightCtrlState c);
        /// <summary>
        /// Called each update in which kOS CONFIG has autopilot disabled, to
        /// ask this controller if it *would have* set the controls had it
        /// been allowed to.  The controller should leave the state neutral
        /// and not set it in this case.  It should also use this as a signal to
        /// clear out integral windup from any PIDs it's using, since its
        /// control is being temporarily suppressed.
        /// </summary>
        /// <param name="c">the flight control state it should try to alter</param>
        /// <returns>true if this controller thinks it *would have* wanted to move the
        /// controls had it not been getting suppressed</returns>
        bool SuppressAutopilot(FlightCtrlState c);
        void EnableControl();
        
        void CopyFrom(IFlightControlParameter origin);
    }
}