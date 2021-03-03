void Ser_print()
{


/*
Serial.print("j");// joint message
Serial.print("\t");
Serial.print(fr_s);//front right
Serial.print("\t");
Serial.print(fl_s);//front left
Serial.print("\t");
Serial.print(rr_s);//rear left
Serial.print("\t");
Serial.print(rl_s);// reasr right
Serial.print("\t");
Serial.print("\n");
*/
Serial.print("b");// joint message
Serial.print("\t");
Serial.print(battery); // ping rear
Serial.print("\t");
Serial.print(rear_dist); // ping rear
/*
Serial.print(ir_l);
Serial.print("\t");
Serial.print(ir_r);
Serial.print("\t");
//Serial.print(rear_bump);
Serial.print("\t");
*/
Serial.print("\t");
Serial.print("\n");


}
