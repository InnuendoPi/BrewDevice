uint16_t getOutputLevel( uint16_t inputLevel ) {
  // https://forum.arduino.cc/index.php?topic=526806.0

  // adjust these 4 constants to suit your application

  // ========================================
  // margin sets the 'stickyness' of the hysteresis or the relucatance to leave the current state.
  // It is measured in units of the the input level. As a guide it is a few percent of the
  // difference between two end points. Don't make the margin too wide or ranges may overlap.
  const uint16_t margin = 10 ;   //  +/- 10

  // set the number of output levels. These are numbered starting from 0.
  const uint16_t numberOfLevelsOutput = 10 ;  // 0..9


  // define input to output conversion table/formula by specifying endpoints of the levels.
  // the number of end points is equal to the number of output levels plus one.
  // in the example below, output level 0 results from an input of between 0 and 112.
  // 1 results from an input of between 113 and 212 etc.
  const uint16_t endPointInput[ numberOfLevelsOutput + 1 ] = { 0, 112, 212, 312, 412, 512, 612, 712, 812, 912, 1023 } ;
  
  // initial output level (usually zero)
  const  uint16_t initialOutputLevel = 0 ;
  // ========================================


  // the current output level is retained for the next calculation.
  // Note: initial value of a static variable is set at compile time.
  static uint16_t currentOutputLevel = initialOutputLevel ;

  // get lower and upper bounds for currentOutputLevel
  uint16_t lb = endPointInput[ currentOutputLevel ] ;
  if ( currentOutputLevel > 0 ) lb -= margin  ;   // subtract margin

  uint16_t ub = endPointInput[ currentOutputLevel + 1 ] ;
  if ( currentOutputLevel < numberOfLevelsOutput ) ub +=  margin  ;  // add margin

  // now test if input is between the outer margins for current output value
  if ( inputLevel < lb || inputLevel > ub ) {
    // determine new output level by scanning endPointInput array
    uint16_t i;
    for ( i = 0 ; i < numberOfLevelsOutput ; i++ ) {
      if ( inputLevel >= endPointInput[ i ] && inputLevel <= endPointInput[ i + 1 ] ) break ;
    }
    currentOutputLevel = i ;
  }
  return currentOutputLevel ;
}
