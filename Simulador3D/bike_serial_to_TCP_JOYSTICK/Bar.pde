class Bar {

  PVector pos;
  int bWidth, bHeight;
  int min, max;

  public Bar(int posX) {
    pos = new PVector(posX, height-50);
    bWidth = 20;
    bHeight = 100-height;
    min = 0;
    max = bHeight;
  }

  public void show(int val) {
    float temp = mapVal(val);
    stroke(200,100);
    noFill();
    rect(pos.x, pos.y, bWidth, bHeight);
        fill(200,80);
    rect(pos.x, pos.y, bWidth, temp);
    fill(200);
    text( val + "km/h", pos.x, pos.y + temp-3);
  }

  private float mapVal(int value) {
    return map(value, min, max, 0, bHeight);
  }

  public void setMinMax(int min, int max) {
    this.min = min;
    this.max = max;
  }
}
