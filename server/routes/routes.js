module.exports = function(app) {
    app.get('/', function(req, res) {
        res.render('index', { title: 'index' });
    });

    app.get('/contactus', function(req, res) {
       // res.render('contactus', { title: 'contactus' });
         response.writeHead(200, { "Content-Type": "text/plain" });
  response.end("Hello World!\n");
    });
}