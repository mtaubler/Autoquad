var x1 = -1;
var x2 = -1;
var y1 = -1;
var y2 = -1;
var i;
var flag = false;


var firstPoint = function(xCoordinate,yCoordinate){
    x1 = xCoordinate;
    y1 = yCoordinate;
}

var secondPoint = function(xCoordinate,yCoordinate){
    x2 = xCoordinate;
    y1 = yCoordinate;
}

var directionVector = function(){
    var dirX = x2 - x1;
    var dirY = y2 - y1;
    var vector = [dirX, dirY];
    console.log("This is the x magnitude: "+ dirX);
    console.log("This is the y magnitude: "+ dirY);
    return vector;
}


for(i=0;i<3;i++){
    console.log("\nThis is how many times we looped: "+ i);
    var randNum1 = Math.floor(Math.random() * 20);
    var randNum2 = Math.floor(Math.random() * 20);
    secondPoint(randNum1,randNum2);
    var dirVector = directionVector();
    if(flag)
        console.log("This is the final directional vector: " + dirVector[0] +","+ dirVector[1]);
    flag = true;
    var randNum3 = Math.floor(Math.random() * 20);
    var randNum4 = Math.floor(Math.random() * 20);
    firstPoint(randNum3, randNum4);
}


