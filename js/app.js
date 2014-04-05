// Foundation JavaScript
// Documentation can be found at: http://foundation.zurb.com/docs

$(document).foundation();


$("#content").click(function() {
    var modal = $('#myModal');
    modal.foundation('reveal', 'open');
    modal.css('width', $('body').width());
    draw();
});
