int numbers[4] = {0,0,0,0};
int number1 = 0;
int number2 = 0;
int number3 = 0;
int number4 = 0;
String inputString = "";

void setup() {
  Serial.begin(9600);
  Serial.println("start! ");
}

void loop() {
  if (Serial.available()) {
    char receivedChar = Serial.read();
    
    if (receivedChar != '\n') {
      inputString += receivedChar;
    } else {
      parseInput();
      inputString = ""; // Clear the input string
    }
  }
}

void parseInput() {
  int parsedNumbers = sscanf(inputString.c_str(), "%d,%d,%d,%d", &numbers[0], &numbers[1], &numbers[2], &numbers[3]);
  
  if (parsedNumbers == 4) {
    // Successfully parsed all four numbers
    Serial.print("Number 1: ");
    Serial.println(numbers[0]);
    Serial.print("Number 2: ");
    Serial.println(numbers[1]);
    Serial.print("Number 3: ");
    Serial.println(numbers[2]);
    Serial.print("Number 4: ");
    Serial.println(numbers[3]);
  } else {
    // Error in parsing data
    Serial.println("Error parsing data.");
  }
}