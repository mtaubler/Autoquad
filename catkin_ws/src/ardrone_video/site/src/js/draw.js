

//creates array of objects with x and y coordinates
var dirVector = [];
var mission = [];
var pt = "something";

if(window.addEventListener) {
    window.addEventListener('load', function () {

        //what selects the draw canvas and allows me to set percent hieght and width for it
        var canvas = document.getElementById('imageView');
        canvas.width = canvas.scrollWidth;
        canvas.height = canvas.scrollHeight;
        var context = canvas.getContext('2d');

        //how thick line is when made by mouse
        var radius = 5;

        //hope users arent holding down mouse as they enter
        var dragging = false; 
        
        //this is what connects two points: gives width to drawn line 
        context.lineWidth = radius * 2;

        //does the erasing of the canvas
        function clearCanvas(canvas,context) {
            context.clearRect(0, 0, canvas.width, canvas.height);
        }        

        
        var computeDistance = function(point1,point2){

            var distBetweenPoints;
            var position1 = point1;
            var position2 = point2;
            
            distBetweenPoints = ((position2.x - position1.x)*(position2.x - position1.x) + (position2.y - position1.y)*(position2.y - position1.y)); 
            return distBetweenPoints
        }

        var breaksToVectors = function(){
            var smallestDistance= canvas.height/20;
            var distance=0;
            var lastPoint=dirVector[0];
            for(var i=0; i<dirVector.length-1; i++){
                var d = computeDistance(dirVector[i],  dirVector[i+1]);
                distance += d;
                
                if(distance > (smallestDistance * smallestDistance) ){
                    var xPos= (dirVector[i+1].x - lastPoint.x)/20;
                    var yPos= (dirVector[i+1].y - lastPoint.y)/20;
                    var nextMission ={x:xPos, y:yPos};
                    
                    mission.push(nextMission);
                    lastPoint=dirVector[i+1];
                    
                    distance=0;
                }
            }
        
            var sum = mission[0];
            //console.log("x:" + sum.x + "  y: " + sum.y+"\n");
            for(var i=0; i<mission.length; i++){
                sum.x += mission[i].x;
                sum.y += mission[i].y;
                console.log("x:" + sum.x + "  y: " + sum.y +"\n");
            }

            return mission;
        }

        //used for finding the displacement vector
        var x1, x2, y1, y2;
        var flag = false;

        //makes the center of the canvas (0,0)
        var cx = canvas.width*.5;
        var cy = canvas.height*(.5);

        //to do: scale pixels to meters --> idk how to do that yet
        //mousemoving event
        var putPoint = function(e){
            if(dragging){
                
                //Draws line from old point to new point = new point = new mouse position
                context.lineTo(e.offsetX, e.offsetY);
                context.stroke();
                
                
                
                //Draws circular points - cur mouse pos, radius size, circumference of point in radians
                context.beginPath();
                context.arc(e.offsetX, e.offsetY, radius, 0, Math.PI * 2);
                context.fill();

                //Creates path after dot = old point = current mouse position
                context.beginPath();
                context.moveTo(e.offsetX, e.offsetY);

                //adds the onscreen mouse coordinates when mousedown on canvas
                document.getElementById('mouseCoors').innerHTML = 'X: ' + (e.offsetX - cx) + ' Y: ' + (cy - e.offsetY);
            
                var point = {x:(e.offsetX - cx), y:(cy - e.offsetY)};

                dirVector.push(point);
                
            }
        }
        
        

        //mousedown event: starts a new path for drawing
        var engage = function(e){
            dragging = true;
            putPoint(e);
        }

        //mouseup event: resets flags and ends current line path
        var disengage = function(){
            dragging = false;
            flag = false;
            //pointCount=0;
            //breaksToVectors();

            //starts from new line every mouseup 
            context.beginPath();
        }

        //mouseleave event: clears the canvas of any drawn lines
        var erase = function(){
            clearCanvas(canvas,context);
        }

        canvas.addEventListener('mousedown',engage);
        canvas.addEventListener('mousemove',putPoint);
        canvas.addEventListener('mouseup',disengage);
        //canvas.addEventListener('mouseleave',erase);
        


        function changeHandler(){
            //Do Something...maybe another function showAlert(), for instance
            if(toggle.checked){
                pt ="tum";
            }
            else{
               //do something else
                pt ="noisy";
            }
         }
        
        document.getElementById('toggle').addEventListener('change', changeHandler);

        document.getElementById("myBtn").addEventListener("click", publishCommands());
	

    }, false); }

    
