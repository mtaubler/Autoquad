            var timerID;
            window.onload = function()
            {
                
                canvas = document.getElementById('myCanvas');
                canvas.width = canvas.scrollWidth;
                canvas.height = canvas.scrollHeight;
                ctx = canvas.getContext('2d');
                
                var video = document.getElementById('video');
                
        
                video.addEventListener('play', function(){
                    
                    timerID = window.setInterval(function(){
                        ctx.drawImage(video,0,0,canvas.width,canvas.height);
                    }, 30);
                });
                
                video.addEventListener('pause', function(){
                    stopTimer();
                });
                
                video.addEventListener('ended', function(){
                    stopTimer();
                });
            };
            
            function stopTimer() {
                window.clearInterval(timerID);
            }
