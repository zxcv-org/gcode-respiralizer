use gcode_respiralizer_lib as gr;
use std::env;
use std::thread;

const STACK_SIZE: usize = 16 * 1024 * 1024;

fn run() {
    let args: Vec<String> = env::args().collect();
    dbg!(&args);

    #[derive(Debug)]
    struct Filenames<'a> {
        fine_reference: &'a str,
        coarse_reference: &'a str,
        // output and coarse_reference can be the same filename; we don't overwrite until we're sure we have complete output
        output: &'a str,
    }

    let filenames = if args.len() == 4 {
        Filenames {
            fine_reference: &args[1],
            coarse_reference: &args[2],
            output: &args[3],
        }
    } else if args.len() == 2 {
        // TODO: if detect fine, save aside; if detect coarse, use saved-aside fine + coarse + output over coarse when sure we have complete output
        eprintln!("in-place / autodetect not yet supported");
        return;
    } else {
        eprintln!("usage:\ngcode_respiralizer autodetect_fine_reference_or_modified_in_place.gcode\nor\ngcode_respiralizer fine_vase_esque_reference.gcode coarse_vase_reference.gcode output.gcode");
        return;
    };

    dbg!(&filenames);

    gr::process_files(
        filenames.fine_reference,
        filenames.coarse_reference,
        filenames.output,
    );
}

fn main() {
    println!("gcode_respiralizer starting");
    // Some threads online recommend changing the default stack size process-wide via linker config
    // but we actually don't need to be boosting the stack size for any threads created by libs,
    // as long as it's just KdTree blowing a smaller stack when called from the primary thread.
    //
    // We only want to change the stack size for the primary thread (not this thread, the thread
    // we're about to create here.
    println!("delegating to separate 'primary' thread with larger stack for KdTree (might still overflow?");
    // Switch threads before we have any data, to avoid needing to worry about Send or Sync.
    let primary = thread::Builder::new()
        .stack_size(STACK_SIZE)
        .spawn(run)
        .expect("thread spawn failed");
    primary
        .join()
        .expect("primary thread join (from main) failed");
}
