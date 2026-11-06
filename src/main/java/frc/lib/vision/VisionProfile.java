package frc.lib.vision;

/**
 * How vision is being used right now. The game decides which one matters most.
 */
public enum VisionProfile {
    /** Know where we are anywhere on the field. Uses every tag. This is the normal mode. */
    LOCALIZATION,

    /**
     * Line up precisely on one scoring spot. Only uses the tags for that spot,
     * and trusts vision more when we're close to them.
     */
    ALIGNMENT
}
